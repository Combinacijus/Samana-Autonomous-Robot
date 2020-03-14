"""
    Gintaras Grebliunas
    combinacijus@gmail.com

    Detects gold bag in a video stream, finds bounding box
    and sends this data via TCP/IP to a server

    NOTE: Before use update DIR_MODEL and DIR_CONFIG
"""

# from imageai.Detection.Custom import CustomObjectDetection
# import cv2 as cv
import time
import socket
import pickle


class Detector:
    """
        Handles object detection
    """

    def __init__(self):
        # NOTE: Change to update model
        self.DIR_MODEL = "detection_model-ex-58--loss-1.79.h5"
        self.DIR_CONFIG = "detection_config.json"
        self.MIN_PROB = 0.2  # NOTE: Change to sensible value

        # Variables
        self.num = 1

        self.init_object_detection()

    def init_object_detection(self):
        # Find usb webcam | NOTE: make sure it's working
        print("Starting video capture...")
        self.cap = cv.VideoCapture(2)
        if self.cap is None or not self.cap.isOpened():
            print("CRITICAL ERROR: CAMERA NOT FOUND!")
        else:
            print("Video capture started")

        print("Initializing object detection model...")
        self.detector = CustomObjectDetection()
        self.detector.setModelTypeAsYOLOv3()
        self.detector.setJsonPath(self.DIR_CONFIG)
        self.detector.setModelPath(self.DIR_MODEL)
        self.detector.loadModel()
        print("Model loaded")

    def detect_objects(self):
        '''
            return: ImageAI detection dictionary of highest probability detection
            or returns None if nothing detected
        '''

        start_t = time.time()  # Timming

        try:
            for i in range(7):  # Cleans buffer and reads current frame
                _, frame = self.cap.read()  # Get new frame from camera

            # Detect from image
            frame_out, detections = self.detector.detectObjectsFromImage(
                input_type="array", input_image=frame, output_type="array", minimum_percentage_probability=self.MIN_PROB)

            # Get highest probability detection
            best_det = None
            if detections:
                best_det = max(detections, key=lambda x: x['percentage_probability'])

            # Print all detections
            # print("All detections:")
            # for det in detections:
            #   self.print_detection(det)
        except Exception as e:
            print("ERROR: failed detection. Skipping frame {}".format(self.num))
            print(e)
        else:
            dt = time.time() - start_t  # Timming
            print("Frame: %2d   dt:%4.4f   FPS: %2.3f" % (self.num, dt, 1 / dt))
            self.num += 1

        return best_det

    def print_detection(self, d):
        if d:
            print(d["name"], " : ", d["percentage_probability"], " : ", d["box_points"])
        else:
            print("Not detected")

    def __del__(self):
        self.cap.release()
        print("Capture realeased")


class TcpClient:
    """
        Handles tcp communication with a server
        Tries to reconnect if disconnected
    """

    def __init__(self):
        # Constants
        self.TCP_IP = '127.0.0.1'
        self.TCP_PORT = 5005
        self.BUFFER_SIZE = 256

        self.wait_for_server()

    def wait_for_server(self):
        """
            Tries every second to connect to the sever
        """
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Main socket
        connected = False
        while not connected:  # Try to connect to the server
            try:
                self.s.connect((self.TCP_IP, self.TCP_PORT))
                connected = True
            except Exception:
                print("Waiting for server at {}:{}".format(self.TCP_IP, self.TCP_PORT))
                time.sleep(1)
            else:
                print("Connected to the server {}:{}".format(self.TCP_IP, self.TCP_PORT))

    def detection_requested(self):
        '''
            return: True if received bool value "True" from the server
        '''
        try:
            data = self.s.recv(self.BUFFER_SIZE)

            if data:
                data = pickle.loads(data)
                print("received: {}".format(data))

            if data is True:
                return True
            else:
                return False

        except Exception:
            print("Server connection lost!")
            self.wait_for_server()
            return False

    def send(self, msg):
        try:
            self.s.send(msg)
        except Exception:
            print("Server connection lost!")
            self.wait_for_server()


if __name__ == "__main__":
    client = TcpClient() # TODO  CHAAAAAAAAAAAAAAAAAAAAAAANGE--------------------------------------------
    # detector = Detector()
    print("Everything working. Responding to requests...")

    while True:  # Respond to detection requests
        try:
            if client.detection_requested():
                # detection = detector.detect_objects()
                # detector.print_detection(detection)

                # Send data to the server
                time.sleep(1)
                detection = "asdasdsa"
                det_data = pickle.dumps(detection, protocol=2)
                client.send(det_data)
            else:  # Detection is not requested. Wait
                time.sleep(0.01)
        except Exception:
            pass  # Hope it won't crash

    print("Program finished.")
