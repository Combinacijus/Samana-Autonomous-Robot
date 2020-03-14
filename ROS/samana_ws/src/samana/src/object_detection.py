#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com

Send out message to TCP/IP client to read camera and detect object
Gets message from client about detected objectss and their position
Transforms screen position to world frame
'''

import rospy
import socket
import pickle
import time
# from std_msgs.msg import UInt8


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
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.TCP_IP, self.TCP_PORT))
        # self.conn = None  # For client connection

        self.wait_for_client()
    

    def wait_for_client(self):
        while True:
            try:
                print("Waiting for client at {}:{}".format(self.TCP_IP, self.TCP_PORT))
                self.s.listen(1)
                self.conn, addr = self.s.accept()
                print('Connected to the client:', addr)
                return
            except Exception:
                print("ERROR: COULD NOT CONNECT TO CLIENT")

    def receive_data(self):
        try:
            data = self.conn.recv(self.BUFFER_SIZE)

            if not data:
                return None
            else:
                return data
        except Exception:
            self.wait_for_client()
            return None

    def send(self, msg):
        try:
            self.conn.send(msg)
        except Exception:
            print("ERROR: Could not send the message: ", msg)
            self.wait_for_client()

if __name__ == '__main__':
    server = TcpServer()

    while True:
        try:
            msg = True
            msg = pickle.dumps(msg)
            server.send(msg)
            print("send", msg)

            data = server.receive_data()
            if data:
                data = pickle.loads(data)
            print("rec: ", data)
            time.sleep(1.2)
        except KeyboardInterrupt:
            print("Program finished")
            break

