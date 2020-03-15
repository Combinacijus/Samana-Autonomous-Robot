#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com

Send out message to TCP/IP client to read camera and detect object
Gets message from client about detected objectss and their position
Transforms screen position to world frame

NOTE: If socket in use run   lsof -i :5005   and check listening processes

Read NOTE: for important places in code
'''

import rospy
import socket
import pickle
import time
import timeout_decorator
# import cv2 as cv
from subprocess import Popen
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

TIMEOUT = 2  # Detection timeout
IMG_W = 640
IMG_H = 480

class TcpServer:
    '''
        Sends requests to the client to detect objects
        and receives detection data
    '''

    def __init__(self):
        # Constants
        self.TCP_IP = '127.0.0.1'
        self.TCP_PORT = 5005
        self.BUFFER_SIZE = 1024

        # Init server
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.TCP_IP, self.TCP_PORT))
        except Exception:
            print("Close image viewer window")
            raise

        self.wait_for_client()

    def wait_for_client(self):
        while True:
            try:
                print("Waiting for client at {}:{}".format(
                    self.TCP_IP, self.TCP_PORT))
                self.socket.listen(1)
                self.conn, addr = self.socket.accept()
                print('Connected to the client:', addr)
                return
            except Exception:
                print("ERROR: COULD NOT CONNECT TO CLIENT")

    @timeout_decorator.timeout(TIMEOUT)
    def receive_data(self):
        data = self.conn.recv(self.BUFFER_SIZE)

        if not data:
            return None
        else:
            return data

    def send(self, msg):
        try:
            self.conn.send(msg)
        except Exception:
            print("ERROR: Could not send the message: ", msg)
            self.wait_for_client()


class DetectionParser:
    def __init__(self):
        self.server = TcpServer()
        self.pickled_data = None
        self.viewer = None

    def send_request(self, debug_image=False):  # NOTE: For seeing detected object image
        '''
            Request contains data of how many seconds are given before timeout
            Also for simplicity if timeout >=1000 it means to enable debug_image
        '''
        msg = TIMEOUT
        if debug_image is True:  # Tell client to enable debug_image
            msg += 1000
        msg = pickle.dumps(msg)
        self.server.send(msg)
        print("SENT: timeout: {}  debug_img: {}".format(TIMEOUT, debug_image))

    def receive_data(self):
        '''
            @return: 
                if detected: pickled imageai dictionary of detection
                not detected: pickled None
                on exception: None
        '''
        try:
            self.pickled_data = self.server.receive_data()
        except KeyboardInterrupt:  # BUG: won't catch
            raise
        except Exception as e:
            print("ERROR: TIMEOUT! FAILED TO RECEIVE DETECTION.")
            print("CHECK SOCKET CONNECTION, TIMEOUT SETTINGS, CAMERA CONNECTION")
            print("OR SERVER IS STILL INITIALIZING")
            print(e)
            self.pickled_data = None
        return self.pickled_data

    def data_debug(self, debug_image=False):
        if self.pickled_data:
            detection = pickle.loads(self.pickled_data)
            if detection:
                print("{d[name]} : {d[percentage_probability]} : {d[box_points]}".format(
                    d=detection))
            else:
                print("No detections")
        else:
            print("No detections")

        # Display image
        if debug_image is True:
            if not self.viewer:
                # NOTE: path to the image
                self.viewer = Popen(["eog", "/home/combinacijus/Documents/SamanaAutonomousRobot/Python/GoldBagDetector/image.jpg"])
        else:
            try:
                self.viewer.terminate()
                self.viewer = None
            except Exception:
                pass


class RosHandler():
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
        '''
            Service callback function for setting debug_image variable
        '''
        if isinstance(req, SetBoolRequest):
            self.debug_image = req.data
            return SetBoolResponse(True, '')
        return SetBoolResponse(False, 'Wrong data type')


if __name__ == '__main__':
    rosh = RosHandler(True)  # ROS handler
    dparser = DetectionParser()

    while True:
        try:
            print("---------------------------")

            dparser.send_request(rosh.debug_image)
            pickled_data = dparser.receive_data()
            if not pickled_data:  # No data received
                continue
            dparser.data_debug(rosh.debug_image)

            data = pickle.loads(pickled_data)
            if data:  # Something detected TODO: continue work
                box = data["box_points"]
                prob = data["percentage_probability"]
                print("box {}".format(box))
                
                cx = (box[0] + box[2]) / 2.0
                cy = (box[1] + box[3]) / 2.0
                dx = (box[2] - box[0])
                dy = (box[3] + box[1])
                center = [cx / IMG_W, cy / IMG_H]  # In unit 
                lower10 = [cx / IMG_W, cy + 0.45*dy]  # In unit. 10% above the bottom so it point more to the base of object
                print("center ({:.2f}, {:.2f}   with prob: {:.2f})".format(center[0], center[1], prob))
                print("center ({:.2f}, {:.2f}".format(lower10[0], lower10[1]))  # TODO: check if works
                # TODO show points in opencv
                # TODO Project camera space to ground plane
            else:  # No detection
                pass

        # Ctrl + C (doesn't work because dparser.receive_data)
        except KeyboardInterrupt:
            print("Program finished")
            break
        except Exception as e:
            print("HOPE NO CRASH: ", e)
