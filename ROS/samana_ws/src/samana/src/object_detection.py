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
import signal
import sys
import socket
import pickle
import time
import timeout_decorator
from subprocess import Popen
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

TIMEOUT = 3  # NOTE: Detection timeout (default 3s)
IMG_W = 640
IMG_H = 480


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
    def __init__(self, debug_image=False):
        self.debug_image = debug_image
        self.init_ros()

    def init_ros(self):
        rospy.init_node('yolo_detector', anonymous=True)

        # Publishers
        # self.teleop_pub = rospy.Publisher('teleop', Teleop, queue_size=10)

        # Messages
        # self.cmd_vel = Twist()

        # Subscribers
        # rospy.Subscriber("rc/modes", RCModes, self.rc_modes_calback)

        # Services
        rospy.Service("set_debug_image", SetBool, self.handle_set_debug_image)

        # Infinite loop
        # self.control_motors()

    def handle_set_debug_image(self, req):
        """
            Service callback function for setting debug_image variable
        """
        if isinstance(req, SetBoolRequest):
            self.debug_image = req.data
            return SetBoolResponse(True, '')
        return SetBoolResponse(False, 'Wrong data type')


def signal_handler(sig, frame):
    sys.exit(0)


# ----------------------------- MAIN PROGRAM ---------------------------
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)  # Needed for Ctrl+C to work
    rosh = RosHandler(debug_image=True)  # NOTE: debug_image change here
    dparser = DetectionParser()

    while True:
        try:
            print("")

            # Get detection data
            dparser.send_request(TIMEOUT, debug_image=rosh.debug_image)
            detection_data = dparser.receive_data()
            dparser.debug_data(rosh.debug_image)
            if not detection_data or detection_data["success"] is False:
                time.sleep(0.5)
                continue

            # Use detection data
            data = detection_data["detection"]
            if data:  # Something detected TODO: continue work
                box = data["box_points"]
                prob = data["percentage_probability"]
                print("box {}".format(box))

                cx = (box[0] + box[2]) / 2.0
                cy = (box[1] + box[3]) / 2.0
                dx = (box[2] - box[0])
                dy = (box[3] + box[1])
                center = [cx / IMG_W, cy / IMG_H]  # In unit
                lower10 = [cx / IMG_W, cy + 0.45 * dy]  # In unit. 10% above the bottom so it point more to the base of object
                print("center ({:.2f}, {:.2f}   with prob: {:.2f})".format(center[0], center[1], prob))
                print("center ({:.2f}, {:.2f})".format(lower10[0], lower10[1]))  # TODO: check if works
                # TODO show points in opencv
                # TODO Project camera space to ground plane
                # TODO Check probability
            else:  # No detection
                pass

            # time.sleep(0.7)  # NOTE: this sleep reduces resource use but lowers frame rate

        except Exception as e:
            print("HOPE NO CRASH: ", e)
            time.sleep(0.05)
