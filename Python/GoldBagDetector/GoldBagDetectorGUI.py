"""
    Gintaras Grebliunas
    combinacijus@gmail.com

    This script draws bounding box around gold bag and calculates
    center position of the box

    NOTE: Before use update DIR_MODEL and DIR_CONFIG
"""

from imageai.Detection.Custom import CustomObjectDetection
import cv2, time
import queue
import threading

DIR_MODEL = "detection_model-ex-58--loss-1.79.h5"
DIR_CONFIG = "detection_config.json"


class BufferlessVideoCapture:
    """
        Discards all previous frames on other threads
        So that only most recent frame is available
    """

    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.q = queue.Queue()

        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

        # read frames as soon as they are available, keeping only most recent one

    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()  # discard previous (unprocessed) frame
                except queue.Empty:
                    pass

            self.q.put(frame)
            # print("t: {:.7f}  FRAME".format(time.time()))

    def read(self):
        # print("t: {:.7f}  READ FRAME".format(time.time()))
        return self.q.get()


# Find usb webcam
# cap = None
# cap_i = 0
# for i in range(1, 11):
#     try:
#         print(i)
#         cap = cv2.VideoCapture(i)
#         _, frame = cap.read()
#         cv2.imshow("aa", frame)
#         cv2.destroyAllWindows()
#         cap_i = i
#         break
#     except Exception:
#         pass

# Use IP camera
cap = BufferlessVideoCapture("http://192.168.8.102:4747/video")

detector = CustomObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setJsonPath(DIR_CONFIG)
detector.setModelPath(DIR_MODEL)
# detector.loadModel()

start_t = time.time()
num = 1
while cv2.waitKey(1) != ord('q'):
    # Timming
    end_t = time.time()
    dt = end_t - start_t
    debug_str = "Frame: %4d   dt: %4.1f ms   FPS: %2.2f   t: %.4f" % (num, dt * 1000, 1 / dt, time.time())
    print(debug_str)
    start_t = time.time()
    num += 1

    try:
        frame = cap.read()  # Get new frame from camera

        # detections = detector.detectObjectsFromImage(input_type="array", input_image=frame,
        #                                              output_image_path="image2new.jpg",
        #                                              minimum_percentage_probability=50)
        # frame_out, detections = detector.detectObjectsFromImage(input_type="array", input_image=frame,
        #                                                         output_type="array", minimum_percentage_probability=1)
        # for det in detections:
        #     print(det["name"], " : ", det["percentage_probability"], " : ", det["box_points"])
    except Exception:
        num -= 1
        print("Error in detection. Skipping frame")
        # break
    else:
        # Debug text on image
        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (5, 25)
        fontScale = 0.55
        color = (0, 0, 255)
        thickness = 2
        frame_out = cv2.putText(frame, debug_str, org, font,
                                fontScale, color, thickness, cv2.LINE_AA)

        cv2.imshow("Gold bag detector", frame_out)
        # cv2.imwrite("file%d.jpg" % num, frame_out)

cap.release()
cv2.destroyAllWindows()
