#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com

Send out message to TCP/IP client to read camera and detect object
Gets message from client about detected objectss and their position
Transforms screen position to world frame

Read NOTE: for important places in code
'''

import rospy
import socket
import pickle
import time
import timeout_decorator
import cv2 as cv
from subprocess import Popen
# from std_msgs.msg import UInt8

TIMEOUT = 2  # Detection timeout

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
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.TCP_IP, self.TCP_PORT))

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
        self.debug_image = True  # NOTE: For seeing detected object image
        self.server = TcpServer()
        self.pickled_data = None
        self.viewer = None

    def send_request(self):
        '''
            Request contains data of how many seconds are given before timeout
            Also for simplicity if timeout >=1000 it means to enable debug_image
        '''
        msg = TIMEOUT
        if self.debug_image is True:  # Tell client to enable debug_image
            msg += 1000
        msg = pickle.dumps(msg)
        self.server.send(msg)
        print("SENT: timeout: {}  debug_img: {}".format(TIMEOUT, self.debug_image))

    def receive_data(self):
        '''
            @return: 
                if detected: pickled imageai dictionary of detection
                not detected: pickled None
                on exception: None
        '''
        try:
            self.pickled_data = self.server.receive_data()
        except Exception as e:
            print("ERROR: TIMEOUT! FAILED TO RECEIVE DETECTION.")
            print("CHECK SOCKET CONNECTION, TIMEOUT SETTINGS, CAMERA CONNECTION")
            print("OR SERVER IS STILL INITIALIZING")
            print(e)
            self.pickled_data = None
        return self.pickled_data

    def data_debug(self):
        if self.pickled_data:
            detection = pickle.loads(self.pickled_data)
            if detection:
                print("{d[name]} : {d[percentage_probability]} : {d[box_points]}".format(d=detection))
            else:
                print("No detections")
        else:
            print("No detections")
        
        # Display image
        if self.debug_image is True:
            if not self.viewer:
                # NOTE: path to the image
                self.viewer = Popen(["eog", "/home/combinacijus/Documents/SamanaAutonomousRobot/Python/GoldBagDetector/image.jpg"])
        else:
            try:
                self.viewer.terminate()
                self.viewer = None
            except Exception:
                pass

if __name__ == '__main__':
    dparser = DetectionParser()

    while True:
        try:
            print("---------------------------")
            dparser.send_request()

            # Receive data
            pickled_data = dparser.receive_data()
            if not pickled_data:
                continue
            
            dparser.data_debug()
        except KeyboardInterrupt:
            print("Program finished")
            break
