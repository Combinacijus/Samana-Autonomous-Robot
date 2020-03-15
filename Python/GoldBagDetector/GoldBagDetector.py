"""
    Gintaras Grebliunas
    combinacijus@gmail.com

    Detects gold bag in a video stream, finds bounding box
    and sends this data via TCP/IP to a server
    
    To run it in docker:
    sudo docker run -it --rm --gpus all --net host --device /dev/video2 -v /home/combinacijus/Documents/SamanaAutonomousRobot/Python/GoldBagDetector/:/notebooks/GoldBagDetector --name imageai combinacijus/imageai:compute3.0

    NOTE: Before use update DIR_MODEL and DIR_CONFIG
"""

from imageai.Detection.Custom import CustomObjectDetection
import cv2 as cv
import time
import socket
import pickle


class Detector:
    """
        Handles object detection using custom YoloV3 model
    """

    def __init__(self):
        self.CAM_INDEX = 2  # Change if using different camera
        # NOTE: Change to update model
        self.DIR_MODEL = "detection_model-ex-58--loss-1.79.h5"
        self.DIR_CONFIG = "detection_config.json"
        self.MIN_PROB = 0.2  # NOTE: Change to sensible value

        # Variables
        self.num = 1

        self.wait_for_camera()
        self.init_object_detection()

    def wait_for_camera(self):
        # Find usb webcam | NOTE: make sure it's working
        while True:
            print("Starting video capture on camera {}...".format(self.CAM_INDEX))
            self.cap = cv.VideoCapture(self.CAM_INDEX)
            if self.cap is None or not self.cap.isOpened():
                print("CRITICAL ERROR: CAMERA {} NOT FOUND!".format(self.CAM_INDEX))
                time.sleep(2)
            else:
                break

        print("Video capture started")

    def init_object_detection(self):
        print("Initializing object detection model...")
        self.detector = CustomObjectDetection()
        self.detector.setModelTypeAsYOLOv3()
        self.detector.setJsonPath(self.DIR_CONFIG)
        self.detector.setModelPath(self.DIR_MODEL)
        self.detector.loadModel()
        print("Model loaded")

    def detect_objects(self, debug_image=False):
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
            print("All detections:")
            for det in detections:
                self.print_detection(det)
        except Exception as e:
            print("ERROR: failed detection. Skipping frame {}".format(self.num))
            print(e)
            self.wait_for_camera()
        else:
            dt = time.time() - start_t  # Timming
            print("Frame: {:2d}   dt:{:4.4f}   FPS: {:2.3f}".format(self.num, dt, 1 / dt))
            self.num += 1

        # Save to file if required
        if debug_image is True:
            # w = int(frame_out.shape[1] * 0.3)
            # h = int(frame_out.shape[0] * 0.3)
            # frame_out = cv.resize(frame_out, (w, h))
            cv.imwrite("image.jpg", frame_out)
            print("Image saved as image.jpg")

        return best_det

    def print_detection(self, data):
        if data:
            print("{d[name]} : {d[percentage_probability]} : {d[box_points]}".format(d=data))
        else:
            print("Not detected")

    def __del__(self):
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
        connected = False
        while not connected:  # Try to connect to the server
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Main socket
                self.socket.connect((self.TCP_IP, self.TCP_PORT))
                connected = True
            except Exception:
                print("Waiting for server at {}:{}".format(self.TCP_IP, self.TCP_PORT))
                time.sleep(1)
            else:
                print("Connected to the server {}:{}".format(self.TCP_IP, self.TCP_PORT))

    def detection_requested(self):
        '''
            @return: timeout in seconds which was sent by server or 0 on exception
        '''
        try:
            data = self.socket.recv(self.BUFFER_SIZE)

            if data:
                data = pickle.loads(data)
                print("Received data: {}".format(data))

            return data

        except Exception:
            print("Server connection lost!")
            self.wait_for_server()
            return 0

    def send(self, msg):
        try:
            self.socket.send(msg)
        except Exception:
            print("Server connection lost!")
            self.wait_for_server()


class TimeIt():
    '''
        Used for measuring time between code in seconds
    '''

    def __init__(self):
        self.start_t = time.time()

    def set_start(self):
        self.start_t = time.time()

    def get_dt(self):
        '''
            @return: elapsed time
        '''
        return time.time() - self.start_t

    def passed(self, t, restart=False):
        '''
            @param restart: if True set_start() when returning true
            @return: if t seconds passed since set_start() return true
        '''
        if self.get_dt() >= t:
            if restart is True:
                self.set_start()
            return True
        return False


if __name__ == "__main__":
    SLEEP_TIME = 0.01

    timeit = TimeIt()
    timeit2 = TimeIt()
    client = TcpClient()
    detector = Detector()

    print("Everything working. Waiting for requests...")

    while True:  # Respond to detection requests
        try:
            print("--------------------")
            data = client.detection_requested()
            if not data:
                client.wait_for_server()
                continue

            # Decode data to timeout and debug_image (same as on server side)
            if data >= 1000:
                debug_image = True
                timeout = data - 1000
            else:
                debug_image = False
                timeout = data

            if timeout <= 0.7 and timeout != 0:  # Warning for the future for setting too low timeout
                print("WARNING: Check if timeout isn't to short. Timeout: {:.3f}sec".format(timeout))

            if timeout != 0:
                timeit.set_start()
                # Do the detection
                detection = detector.detect_objects(debug_image=debug_image)  # TODO set to false
                # detector.print_detection(detection)

                # Send data to the server if timeout not exceeded
                if not timeit.passed(timeout - SLEEP_TIME * 2):
                    det_data = pickle.dumps(detection, protocol=2)
                    client.send(det_data)
                    print("SENT: in {:4.2f}sec. Data: {}".format(timeit.get_dt(), detection))
                else:  # Timeout
                    print("WARNING: Detection timeout! Detection time: {:.2f}".format(timeit.get_dt()))
            else:  # Detection is not requested. Wait
                time.sleep(SLEEP_TIME)
                if timeit2.passed(2, True):
                    print("Detection is not requested. Waiting...")

        except Exception as e:
            # Hope it won't crash
            print("HOPE NO CRASH: ", e)

    print("Program finished.")
