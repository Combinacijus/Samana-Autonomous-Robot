"""
    Gintaras Grebliunas
    combinacijus@gmail.com

    Detects gold bag in a video stream, finds bounding box
    and sends bounding box data via TCP/IP to a server node
    
    To run it in docker:
    sudo docker run -it --rm --gpus all --net host -v /home/combinacijus/Documents/SamanaAutonomousRobot/Python/GoldBagDetector/:/notebooks/GoldBagDetector -v /home/combinacijus/Desktop/ros-logs/:/home/combinacijus/Desktop/ros-logs/ -w /notebooks/GoldBagDetector --name imageai combinacijus/imageai:compute3.0
    python GoldBagDetector.py

    # Run ROS script (TCP/IP server side)
    # Set laptop to performance mode
    feh -R 0.7 /home/combinacijus/Documents/SamanaAutonomousRobot/Python/GoldBagDetector/image.jpg  # For image debugging

    NOTE: Before use update DIR_MODEL and DIR_CONFIG
"""

import cv2
import numpy
import time
import datetime
import socket
import pickle
import queue
import threading
import os
import requests

FAST_DEBUG = False

if FAST_DEBUG is False:
    from imageai.Detection.Custom import CustomObjectDetection


class BufferlessVideoCapture:
    """
        Discards all previous frames on other threads
        So that only most recent frame is available
        Also reconnects to camera if disconnected
    """

    def __init__(self, cam_addr, cam_force_addr=None, cam_focus_addr=None):
        """
        :param cam_addr: ip address of the camera feed
        :param cam_force_addr: ip address to disconnect other clients (forcefully take over)
        """
        self.cam_addr = cam_addr
        self.cam_force_addr = cam_force_addr
        self.cam_focus_addr = cam_focus_addr
        self.q = queue.Queue()
        self.cap = None

        self.reconnect_camera()

        if self.cam_focus_addr is not None:
            t1 = threading.Thread(target=self.auto_focus, daemon=True)
            t1.start()

        t2 = threading.Thread(target=self.ensure_latest_frame, daemon=True)
        t2.start()

    def reconnect_camera(self):
        while True:
            try:
                if self.cam_force_addr is not None:
                    requests.get(self.cam_force_addr)

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

    def auto_focus(self):
        while True:
            try:
                requests.get(self.cam_focus_addr)
            except Exception:
                pass
            time.sleep(5)


def datetime_str():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")


class ImageLogger:
    def __init__(self, dir_logs="/home/combinacijus/Desktop/ros-logs/"):  # NOTE: default logs directory
        """
        :param dir_logs: Base directory for all ros logs
        """
        self.session_dir = "{}images/{}/".format(dir_logs, datetime_str())
        self.jpeg_quality = 60  # 1..100
        self.count = 1

        self.create_dir()

    def create_dir(self):
        try:
            os.makedirs(self.session_dir)
            print("ImageLogger created directory at {}".format(self.session_dir))
        except OSError as e:
            print("LogImage create_dir ERROR: {}".format(e))

    def log(self, img, postfix=""):
        if not isinstance(img, numpy.ndarray):
            print("ImageLogger ERROR: NOT AN IMAGE")
            return

        file_path = "{0}{1}_{2}{3}.jpg".format(self.session_dir, datetime_str(), self.count, postfix)
        cv2.imwrite(file_path, img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        print("Log image saved: " + file_path)
        self.count += 1


class TcpClient:
    """
        Handles tcp communication with a server
        Tries to reconnect if disconnected
    """

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.BUFFER_SIZE = 256
        self.socket = None  # Initialized in connect_to_server()

        self.connect_to_server()

    def connect_to_server(self):
        """
            Tries every second to connect to the sever
        """
        while True:  # Try to connect to the server
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.ip, self.port))
            except Exception:
                print("Waiting for TCP/IP server from ROS side at {}:{}".format(self.ip, self.port))
                time.sleep(1)
            else:
                print("Connected to the server {}:{}".format(self.ip, self.port))
                break

    def get_data(self):
        """
            :return: Data received form the server
        """
        try:
            data = self.socket.recv(self.BUFFER_SIZE)
            if not data:
                self.connect_to_server()
            return data
        except Exception:
            print("Server connection lost!")
            self.connect_to_server()
            return None

    def send(self, msg):
        try:
            self.socket.send(msg)
        except Exception:
            print("Server connection lost!")
            self.connect_to_server()


class Detector:
    """
        Handles object detection using custom YoloV3 model
    """

    def __init__(self):
        self.BASE_IP = "http://192.168.8.101:4747/"  # For home wifi
        # self.BASE_IP = "http://10.42.0.32:4747/"  # For hotspot connection
        self.CAM_ADDR = self.BASE_IP + "video"  # NOTE: might change
        self.CAM_FORCE_ADDR = self.BASE_IP + "override"  # NOTE: might change
        self.CAM_FOCUS_ADDR = self.BASE_IP + "cam/1/af"  # NOTE: might change
        self.DIR_MODEL = "detection_model-ex-016--loss-0001.398.h5"  # TODO NOTE: might change
        self.DIR_CONFIG = "detection_config.json"  # NOTE: might change
        self.MIN_PROB = 30  # TODO NOTE: Change to sensible value to prevent false positives

        self.cap = None
        self.count = 1
        self.detector = None
        self.frame_out = None
        self.frame = None
        self.detections = {}
        self.best_detection = None
        self.success = None

        self.image_logger = ImageLogger()
        self.cap = BufferlessVideoCapture(self.CAM_ADDR, self.CAM_FORCE_ADDR, self.CAM_FOCUS_ADDR)

        if FAST_DEBUG is False:
            self.init_object_detection()
        else:
            print("FAST DEBUG: detector loading skipped")

    def init_object_detection(self):
        print("Initializing object detection model: {}".format(self.DIR_MODEL))
        self.detector = CustomObjectDetection()
        self.detector.setModelTypeAsYOLOv3()
        self.detector.setJsonPath(self.DIR_CONFIG)
        self.detector.setModelPath(self.DIR_MODEL)
        self.detector.loadModel()
        print("Model loaded")

    def detect_objects(self):
        """
        :return: Success(bool), ImageAI detection dictionary of highest probability detection, time when frame was acquired
            or returns None if nothing detected
        """
        self.best_detection = None
        self.detections = None
        self.frame_out = None
        self.frame = None
        frame_time = None
        self.success = True

        self.frame, frame_time = self.cap.read()  # Get new frame from camera
        if self.frame is not None:
            if FAST_DEBUG is True:
                print("FAST DEBUG: detection skipped")
                time.sleep(0.8)
            else:
                self.frame_out, self.detections = self.detector.detectObjectsFromImage(input_type="array", input_image=self.frame,
                                                                                       output_type="array",
                                                                                       minimum_percentage_probability=self.MIN_PROB)
            self.count += 1

        # Get highest probability detection
        if self.detections:
            self.best_detection = max(self.detections, key=lambda x: x["percentage_probability"])
            self.best_detection["total_detections_number"] = len(self.detections)

        if self.frame is None:
            self.success = False

        return self.success, self.best_detection, frame_time, self.detections

    def log_detections(self, debug_image, text=None):
        """
        Logs/Saves frame with detection boxes or if no detection just frame without boxes
        :param debug_image: if true saves latest frame to disk
        :param text: text to write on image
        """
        if self.success or (FAST_DEBUG and self.frame is not None):
            frame_to_save = None
            if self.best_detection and self.frame_out is not None:
                # Draw best detection in green
                d = self.best_detection["box_points"]
                frame_to_save = cv2.rectangle(self.frame_out, (d[0], d[1]), (d[2], d[3]), (0, 255, 0), 2)
                frame_to_save = cv2.circle(frame_to_save, ((d[0] + d[2]) // 2, (d[1] + d[3]) // 2), 5, color=(0, 255, 0), thickness=1)
            else:
                frame_to_save = self.frame

            if isinstance(text, str):
                frame_to_save = cv2.putText(frame_to_save, text, (5, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)

            self.image_logger.log(frame_to_save)

            if debug_image is True:
                cv2.imwrite("latest_detection_debug.jpg", frame_to_save)
                print("Image saved as latest_detection_debug.jpg")

    def print_detections(self):
        if self.detections:
            for det in self.detections:
                print("{d[name]} : {d[percentage_probability]} : {d[box_points]}".format(d=det))
        else:
            print("No object detected")


class TimeIt:
    """
        Used for measuring time between code in seconds
    """

    def __init__(self):
        self.start_t = time.time()

    def set_start(self):
        self.start_t = time.time()

    def dt(self):
        """
            :return: Elapsed time in seconds
        """
        return time.time() - self.start_t

    def fps(self):
        """
        :return: Frames per second
        """
        return 1 / self.dt()

    def passed(self, t, restart_after_true=False):
        """
        :param t: Seconds
        :param restart_after_true: If True set_start() when returning true
        :return: If t seconds passed since set_start() return true
        """

        if self.dt() >= t:
            if restart_after_true is True:
                self.set_start()
            return True
        return False


class MainWorker:
    def __init__(self):
        self.SLEEP_TIME = 0.01

        self.timeout = None
        self.debug_image = None
        self.success = None
        self.detection = None
        self.frame_time = None
        self.all_detections = None

        self.timeit_fps = TimeIt()
        self.timeit_idle = TimeIt()
        self.timeit_timeout = TimeIt()
        self.client = TcpClient(ip='127.0.0.1', port=5005)
        self.detector = Detector()

    def respond_to_requests(self):
        if not self.is_detection_requested():
            time.sleep(self.SLEEP_TIME)
            if self.timeit_idle.passed(2, restart_after_true=True):
                print("Detection is not requested. Waiting...")
            return

        self.timeit_timeout.set_start()
        self.detection_step()

        timedout = self.timeit_timeout.passed(self.timeout - self.SLEEP_TIME * 2)

        self.data_sending_step(timedout)
        self.logging_step(timedout)

        print("----")

    def is_detection_requested(self):
        data = self.client.get_data()
        self.timeout, self.debug_image = self.decode_data(data)

        if self.timeout is None:
            return False
        return True

    def decode_data(self, data_pickled):
        """
        Decodes pickled dictionary to timeout(float) and debug(bool)
        :param data_pickled: Pickled dictionary
        :return: (timeout(float in sec), debug_imaged(bool))
        """

        if data_pickled:
            try:
                data = pickle.loads(data_pickled)
                timeout = data["timeout"]
                debug_image = data["debug_image"]

                # Warning for the future for setting too low timeout
                if timeout <= 1.0 and timeout != 0:
                    print("WARNING: Check if timeout isn't to short. Timeout: {:.3f}sec".format(timeout))

                return timeout, debug_image
            except Exception as e:
                print("BAD PICKLED DATA: {} ERROR: {}".format(data_pickled, e))

        return None, None

    def detection_step(self):
        self.timeit_fps = TimeIt()
        self.success, self.detection, self.frame_time, self.all_detections = self.detector.detect_objects()

        if self.success is False:
            print("WARNING: Frame capture failed!")

    def data_sending_step(self, timedout):
        if not timedout:
            data_to_send, data_not_pickled = self.encode_data()
            self.client.send(data_to_send)
            print("SENT: in {:4.2f}sec. t: {:.7f}\nData: {}"
                  .format(self.timeit_timeout.dt(), time.time(), data_not_pickled))
        else:  # Timeout
            print("WARNING: Detection timeout! Detection time > Timeout: {:.2f}sec > {:.2f}sec"
                  .format(self.timeit_timeout.dt(), self.timeout))

    def encode_data(self):
        """
        :return: Pickled data of merge dictionary {frame_time, detection}
        """
        data_dict = {"success": self.success, "frame_time": self.frame_time}

        if self.detection is None:
            data_dict.update({"detection": None})
        else:
            data_dict.update({"detection": self.detection})

        if self.all_detections is None:
            data_dict.update({"all_detections": None})
        else:
            data_dict.update({"all_detections": self.all_detections})

        data_pickled = pickle.dumps(data_dict, protocol=2)

        return data_pickled, data_dict

    def logging_step(self, timedout):
        if self.success is False:
            return

        self.detector.print_detections()
        if self.success or FAST_DEBUG:
            text = "TIMEOUT! {:.2f}".format(self.timeit_fps.dt()) if timedout else None
            self.detector.log_detections(debug_image=self.debug_image, text=text)
        print("Frame: {:2d}   dt: {:4.4f}   FPS: {:2.3f}"
              .format(self.detector.count, self.timeit_fps.dt(), self.timeit_fps.fps()))


if __name__ == "__main__":
    main_worker = MainWorker()

    print("-----------------------------------------------")
    print("| Everything working. Waiting for requests... |")
    print("-----------------------------------------------")

    while True:
        try:
            main_worker.respond_to_requests()
        except Exception as e:
            # Hope it won't crash because it's critical
            print("HOPE NO CRASH: ", e)
