#!/usr/bin/env python
"""
Gintaras Grebliunas
combinacijus@gmail.com

Send out message to TCP/IP client to read camera and detect object
Gets message from client about detected objects and their position

To run object detection script in docker:
    sudo docker run -it --rm --gpus all --net host -v /home/combinacijus/Documents/SamanaAutonomousRobot/Python/GoldBagDetector/:/notebooks/GoldBagDetector -v /home/combinacijus/Desktop/ros-logs/:/home/combinacijus/Desktop/ros-logs/ -w /notebooks/GoldBagDetector --name imageai combinacijus/imageai:compute3.0
    python GoldBagDetector.py

    - Connect to this computer hotspot
    - Run ROS script (TCP/IP server side)
    - Set laptop to performance mode
    feh -R 0.7 /home/combinacijus/Documents/SamanaAutonomousRobot/Python/GoldBagDetector/latest_detection_debug.jpg  # For image debugging

NOTE: If socket in use run   lsof -i :5005   and check listening processes and sudo kill -9 <PID>

Read NOTE: for important places in code
"""

import rospy
import rospkg
import numpy as np
import cv2
import signal
import sys
import socket
import pickle
import time
import queue
import math
import threading
import timeout_decorator
from geometry_msgs.msg import PointStamped
from collections import namedtuple
from subprocess import Popen
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from shapely.geometry import Polygon, Point


FAKE_DETECTOR = False  # NOTE: if true uses faster face detector TODO set to false for event!
TIMEOUT = 5  # NOTE: Detection timeout (default 5s)
IMG_W = 640
IMG_H = 480

BoundingBox = namedtuple("BoumdingBox", ["x1", "y1", "x2", "y2"])


class TcpServer:
    """
        Sends requests to the client to detect objects
        and receives detection data
    """

    def __init__(self, ip, port):
        # Constants
        self.ip = ip
        self.port = port
        self.conn = None
        self.BUFFER_SIZE = 1024

        # Init server
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.ip, self.port))
        except Exception:
            print("Close image viewer window")
            raise

        self.wait_for_client()

    def wait_for_client(self):
        while True:
            try:
                print("Waiting for client at {}:{}...".format(self.ip, self.port))
                self.socket.listen(1)
                self.conn, address = self.socket.accept()
                print('Connected to the client: ', address)
                return
            except Exception as e:
                print("ERROR: COULD NOT CONNECT TO THE CLIENT. {}".format(e))

    @timeout_decorator.timeout(TIMEOUT, exception_message="TCP client timed out")
    def receive_data(self):
        """
        :return: on fail: empty string
        """
        return self.conn.recv(self.BUFFER_SIZE)

    def send(self, msg):
        """
        :return: Success status
        """
        try:
            self.conn.send(msg)
            return True
        except Exception:
            print("ERROR: Could not send the message: {}".format(msg))
            self.wait_for_client()
            return False


class DetectionParser:
    def __init__(self):
        self.server = TcpServer('127.0.0.1', 5005)  # Communicating with a Docker container which does object detection
        self.pickled_data = None
        self.detection_data = None
        self.gui_viewer = None

    def send_request(self, timeout, debug_image=False):  # debug_image for saving detected object image
        """
        :param timeout: in seconds
        :param debug_image: if true save send flag to save debug image to disk for viewing in GUI
        :return: Success status
        """
        msg = {"timeout": timeout, "debug_image": debug_image}
        msg_pickled = pickle.dumps(msg)
        if self.server.send(msg_pickled):
            print("SENT DATA: {}".format(msg))
            return True
        return False

    def receive_data(self):
        """
            :return: dictionary with received data
                     or None on fail
        """
        self.detection_data = None
        self.pickled_data = None

        try:
            self.pickled_data = self.server.receive_data()
            if not self.pickled_data:
                raise ValueError("ERROR: Failed to receive detection")

            self.detection_data = pickle.loads(self.pickled_data)
        except ValueError as e:
            print(e)
            print("Possible causes: lost socket connection, client lost camera connection or client is still initializing")
        except timeout_decorator.timeout_decorator.TimeoutError:
            print("WARNING: Detector timed out. Check your CAMERA connection and TIMEOUT period!")
        except Exception as e:
            print("ERROR receive_data(): {}".format(e))

        return self.detection_data

    def debug_data(self, debug_image=False):
        if self.detection_data:
            print("RECEIVED: {}".format(self.detection_data))

            if self.detection_data["success"] is False:
                print("Detector failed! Check your camera!")
                return

            if self.detection_data["detection"]:
                print("{d[name]} : {d[percentage_probability]} : {d[box_points]}".format(d=self.detection_data["detection"]))
            else:
                print("No detections")

        # Display image
        if debug_image is True:
            if not self.gui_viewer:
                # NOTE: path to the image
                self.gui_viewer = Popen(["feh", "-R 0.4",
                                         "/home/combinacijus/Documents/SamanaAutonomousRobot/Python/GoldBagDetector/latest_detection_debug.jpg"])
        elif self.gui_viewer is not None:
            try:
                self.gui_viewer.terminate()
                self.gui_viewer = None
            except Exception:
                pass


class RosHandler:
    def __init__(self, debug_image=False, detector_enabled=False):
        self.debug_image = debug_image
        self.detector_enabled = detector_enabled
        self.init_ros()

    def init_ros(self):
        rospy.init_node('yolo_detector', anonymous=True)

        # Publishers
        self.bag_pos_pub = rospy.Publisher("bag_point", PointStamped, queue_size=1)

        # Messages
        self.bag_pos_msg = PointStamped()

        # Services
        rospy.Service("set_debug_image", SetBool, self.handle_set_debug_image)
        rospy.Service("enable_detector", SetBool, self.handle_enable_detector)

    def handle_set_debug_image(self, req):
        """
            Service callback function for setting debug_image variable
        """
        if isinstance(req, SetBoolRequest):
            self.debug_image = req.data
            return SetBoolResponse(True, '')
        return SetBoolResponse(False, 'Wrong data type')

    def handle_enable_detector(self, req):
        """
            Service callback function for enabling object detector
        """
        if isinstance(req, SetBoolRequest):
            self.detector_enabled = req.data
            return SetBoolResponse(True, '')
        return SetBoolResponse(False, 'Wrong data type')

    def handle_detection_box(self, box, hom, frame_time=None):
        """ 
        Publishes detected box base point to ROS topic in frame (x - forward, y - left)
        :param box: tuple(x1, y1, x2, y2) in screen space
        """
        p = hom.s2w(box_to_point(box))

        if frame_time:
            self.bag_pos_msg.header.stamp.secs = int(frame_time)
            self.bag_pos_msg.header.stamp.nsecs = int(int(frame_time * 1e9) % 1e9)
        else:
            self.bag_pos_msg.header.stamp = rospy.Time.now()
        self.bag_pos_msg.header.frame_id = "/base_link"
        self.bag_pos_msg.point.x = p[1]  # Transformed to (x - forward, y - left)
        self.bag_pos_msg.point.y = -p[0]  # Transformed to (x - forward, y - left)

        self.bag_pos_pub.publish(self.bag_pos_msg)


def signal_handler(sig, frame):
    sys.exit(0)


class BufferlessVideoCapture:
    """
        Discards all previous frames on other threads
        So that only most recent frame is available
        Also reconnects to camera if disconnected
    """

    def __init__(self, cam_addr, cam_force_addr=None):
        """
        :param cam_addr: ip address of the camera feed
        """
        self.cam_addr = cam_addr
        self.q = queue.Queue()
        self.cap = None

        self.reconnect_camera()

        t2 = threading.Thread(target=self.ensure_latest_frame)  # error for daemon=True?
        t2.start()

    def reconnect_camera(self):
        while True:
            try:
                self.cap = cv2.VideoCapture(self.cam_addr)

                print("Connected to a camera: {}".format(self.cam_addr))
                break
            except Exception as e:
                print("Could not connect to a camera: {}. {}".format(self.cam_addr, e))
                time.sleep(1)

    def ensure_latest_frame(self):
        """
        Run from a separate thread.
        Read frames as soon as they are available, keeping only most recent one in a queue
        """

        while True:
            ret, frame = self.cap.read()

            if ret is False:
                self.reconnect_camera()

            self.empty_queue_buffer()

            self.q.put((frame, time.time()))

    def read(self):
        """
        :return: Image from camera.
                 Time when image was read from camera
        """
        frame = None
        frame_time = None

        try:
            self.empty_queue_buffer()  # To ensure getting recent frame. Oldest possible frame will be withing timeout period
            frame, frame_time = self.q.get(block=True, timeout=0.3)
        except queue.Empty:
            pass

        return frame, frame_time

    def empty_queue_buffer(self):
        if not self.q.empty():
            try:
                self.q.get_nowait()  # discard previous (unprocessed) frame
            except queue.Empty:
                pass


class Homography():
    def __init__(self):  # NOTE: Homography tuning.
        # PTS_SCREEN screen coordinates and PTS_WORLD base_link/real world equivalent coordinates
        # PTS_SCREEN = np.array([[2219, 2256], [500, 911], [3263, 965], [1515, 371], [2939, 1881], [2877, 731], [1765, 581], [1029, 1422]])  # High-res 4000x3000
        # NOTE: this calibration is getting worse at the edges (~8cm off) probably should recalibrate for better performance
        PTS_SCREEN = np.array([[355.04, 360.96], [80, 145.76], [522.08, 154.4], [242.4, 59.36], [470.24, 300.96],
                               [460.32, 116.96], [282.4, 92.96], [164.64, 227.52]])  # TODO: change Low-res 640x480
        PTS_WORLD = np.array([[0.1, 0.241], [-0.4, 0.797], [0.50, 0.78], [-0.1, 1.146], [0.3, 0.418], [0.4, 0.91], [0, 0.998], [-0.2, 0.568]])
        self.screen_to_world_tf, status = cv2.findHomography(PTS_SCREEN, PTS_WORLD)  # Transform matrix
        self.world_to_screen_tf = np.linalg.pinv(self.screen_to_world_tf)  # Transform matrix

    def s2w(self, point):
        """ Transform point from screen to real world ground plane (float float)"""
        point = np.array([[list(point)]], dtype='float32')
        p = cv2.perspectiveTransform(point, self.screen_to_world_tf)[0][0]
        return p

    def w2s(self, point):
        """ Transform point from real world ground plane to screen (int int)"""
        point = np.array([[list(point)]], dtype='float32')
        p = cv2.perspectiveTransform(point, self.world_to_screen_tf)[0][0]
        p = tuple(int(x) for x in p)
        return p


class FaceDetector():
    """ Used as fake bag detector for tuning """

    def __init__(self):
        cascPath = rospkg.RosPack().get_path("samana") + "/config/haarcascade_frontalface_default.xml"
        self.face_cascade = cv2.CascadeClassifier(cascPath)

    def detect_face(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

        if faces is not None and len(faces) > 0:
            box = BoundingBox(faces[0][0], faces[0][1], faces[0][0] + faces[0][2], faces[0][1] + faces[0][3])
            return box

        return None


class DisplayFunctions():
    def __init__(self):
        pass

    def draw_ground_grid(self, hom, img):
        """
            Draws grid and angle lines
        """
        for x in range(-7, 11):  # Vertical lines
            x = x / 10.0
            img = cv2.line(img, hom.w2s((x, 0.0)), hom.w2s((x, 1.5)), (100, 200, 100), 1)
            if x == 0:
                img = cv2.line(img, hom.w2s((x, 0.0)), hom.w2s((x, 1.5)), 255, 1)

        for y in range(-5, 15):  # Horizontal lines
            y = y / 10.0
            img = cv2.line(img, hom.w2s((-0.7, y)), hom.w2s((1.0, y)), (100, 200, 100), 1)
            if x == 0:
                img = cv2.line(img, p1, p2, 255, 1)

            img = cv2.putText(img, str(y), hom.w2s((0.0, y)), cv2.FONT_HERSHEY_SIMPLEX,
                              0.5, (255, 0, 0), 1, cv2.LINE_AA)

        for phi in range(-7, 8):  # Angle lines
            phi = phi * 10.0 + 90

            x = 2 * math.cos(math.radians(phi))
            y = 2 * math.sin(math.radians(phi))

            xt = 0.15 * math.cos(math.radians(phi))
            yt = 0.15 * math.sin(math.radians(phi))

            img = cv2.line(img, hom.w2s((0, 0)), hom.w2s((x, y)), (200, 100, 100), 1)
            img = cv2.putText(img, str(int(phi-90)), hom.w2s((xt, yt)), cv2.FONT_HERSHEY_SIMPLEX,
                              0.3, (255, 0, 0), 1, cv2.LINE_AA)

    def draw_bounding_box(self, img, box, color=(0, 255, 0)):
        if box is not None:
            cv2.rectangle(img, (box.x1, box.y1), (box.x2, box.y2), color, 2)

    def draw_point(self, img, hom, p, coords=True):
        """
            :param img: opecv numpy image array
            :param hom: homography object for screen to world and reverse transformations
            :param p: point tuple (x-right, y-forward)
            :param coords: if True draws coordinates next to point
        """
        color = (0, 230, 0)
        img = cv2.circle(img, p, 6, color, 2)
        if coords:
            pw = hom.s2w(p)
            yaw = math.degrees(math.atan2(pw[1], pw[0])) - 90

            text_pts_w = "({:2.2f}x, {:2.2f}y, {:2.1f}deg)".format(pw[1], -pw[0], yaw)  # Transformed to (x - forward, y - left)
            text_pts_s = "({:2.0f}x, {:2.0f}y)".format(p[1], p[0])  # Transformed to (x - forward, y - left)

            img = cv2.putText(img, text_pts_w, (p[0] + 10, p[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)  # Outline
            img = cv2.putText(img, text_pts_w, (p[0] + 10, p[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            img = cv2.putText(img, text_pts_s, (p[0] + 10, p[1] + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)  # Outline
            img = cv2.putText(img, text_pts_s, (p[0] + 10, p[1] + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
        
    def draw_area_text(self, img, hom, point):
        text = "Other"

        # Areas of action  TODO clean up
        area_drive_fwd = Polygon([(1.2, -0.9), (1.6, -0.9), (1.6, 1.1), (1.2, 1.1)])
        area_turn_cw = Polygon([(0.55, 0.6), (1.2, 0.6), (1.2, 1.0), (0.55, 1.0)])
        area_turn_ccw = Polygon([(0.55, -0.4), (0.55, -0.8), (1.2, -0.8), (1.2, -0.4)])
        area_drive_back1 = Polygon([(0, -0.2), (0, -0.5), (0.55, -0.5), (0.55, -0.2)])
        area_drive_back2 = Polygon([(0, 0.2), (0.55, 0.2), (0.55, 0.7), (0, 0.7)])
        area_grab = Polygon([(0.25, -0.03), (0.3, -0.03), (0.3, 0.03), (0.25, 0.03)])


        # Areas visualization by point grid
        if False:
            mult = 1
            for x in range(-8*mult, 12*mult):
                x = x / 10.0 / mult
                for y in range(0, 15*mult):
                    y = y / 10.0 / mult

                    pw = (x, y)
                    p = Point(pw[1], pw[0])
                    ps = hom.w2s(pw)

                    img = cv2.circle(img, ps, 3, (0, 255, 0), 2)
                    
                    count = 0
                    color = (0, 0, 0)
                    if area_drive_fwd.contains(p):
                        color = (100, 0, 200)
                        count += 1
                    if area_turn_cw.contains(p):
                        color = (200, 0, 200)
                        count += 1
                    if area_turn_ccw.contains(p):
                        color = (200, 50, 200)
                        count += 1
                    if area_drive_back1.contains(p) or area_drive_back2.contains(p):
                        color = (50, 50, 230)
                        count += 1
                    if area_grab.contains(p):
                        color = (0, 255, 0)
                        count += 1

                    if count > 1:
                        print("AREAS OVERLAP")

                    img = cv2.circle(img, ps, 3, color, 2)

        p = hom.s2w(point)
        p = Point(p[1], p[0])

        # print("point {}, polygon: {}".format(p, area_drive_fwd))

        if area_drive_fwd.contains(p):
            text = "Drive forward"
        elif area_turn_cw.contains(p):
            text = "Turn CW"
        elif area_turn_ccw.contains(p):
            text = "Turn CCW"
        elif area_drive_back1.contains(p) or area_drive_back2.contains(p):
            text = "Drive backward"
        elif area_grab.contains(p):
            text = "GRAB"

        img = cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)  # Outline
        img = cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 50, 50), 2, cv2.LINE_AA)

    
def box_to_point(box, ratio_y=0.5):  # TODO: change default to bag default
    """
    :param box: tuple (x1, y1, x2, y2)
    :param ratio_y: at what fraction from the bottom to return the point (0.5 = center point, 0.0 = bottom point)
    :return: (tuple) center point or point at specified y fraction
    """

    cx = (box.x1 + box.x2) / 2.0
    cy = (box.y1 + box.y2) / 2.0
    # dx = abs(box.x2 - box.x1)
    dy = abs(box.y2 - box.y1)

    center = (cx, cy)  # In unit
    point = (int(cx), int(cy + (0.5 - ratio_y) * dy))  # In unit. ratio_y*100% above the bottom

    return point

# ----------------------------- MAIN PROGRAM ---------------------------


def main():
    signal.signal(signal.SIGINT, signal_handler)  # Needed for Ctrl+C to work
    rosh = RosHandler(debug_image=True)  # NOTE: debug_image change here
    disp = DisplayFunctions()
    hom = Homography()  # For point transformation between screen and ground plane 

    if FAKE_DETECTOR is True:  # Faking bag detector (fast) and with visualization
        face_det = FaceDetector()
        cap = BufferlessVideoCapture("http://10.42.0.32:4747/video")
        # cap = BufferlessVideoCapture("http://192.168.0.100:4747/video")

        while(True):
            time.sleep(0.4)  # To save some CPU time
            img, frame_time = cap.read()
            if img is None:
                continue

            box = face_det.detect_face(img)
            disp.draw_ground_grid(hom, img)

            if box is not None:
                rosh.handle_detection_box(box, hom, frame_time)
                disp.draw_bounding_box(img, box)
                disp.draw_point(img, hom, box_to_point(box), coords=True)
                disp.draw_area_text(img, hom, box_to_point(box))

            cv2.imshow("Camera view", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    else:  # Using real bag detector (slow)
        dparser = DetectionParser()

        while True:
            try:
                SLEEP = 0.3  # NOTE: tuning to reduce resource use but lowers frame rate

                # Do detection only if enabled
                if rosh.detector_enabled is True:
                    print("")

                    # Get detection data
                    dparser.send_request(TIMEOUT, debug_image=rosh.debug_image)  # Request detection
                    detection_data = dparser.receive_data()  # Receive data
                    dparser.debug_data(rosh.debug_image)  # Print debug info about data
                    if not detection_data or detection_data["success"] is False:  # No or bag data
                        time.sleep(SLEEP)
                        continue

                    # Use detection data
                    data = detection_data["detection"]
                    if data:  # Something detected
                        bp = data["box_points"]
                        box = BoundingBox(bp[0], bp[1], bp[2], bp[3])
                        prob = data["percentage_probability"]
                        print("Box {}\n Probability: {} frame_time: {}".format(box, prob, detection_data["frame_time"]))  # TODO remove

                        rosh.handle_detection_box(box, hom, frame_time=detection_data["frame_time"])
                    else:  # No detection
                        pass

                time.sleep(SLEEP)

            except Exception as e:
                print("HOPE NO CRASH: ", e)
                time.sleep(0.05)


if __name__ == '__main__':
    main()
