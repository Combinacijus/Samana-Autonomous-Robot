"""
    Gintaras Grebliunas
    combinacijus@gmail.com

    This script draws bounding box around gold bag and calculates
    center position of the box

    NOTE: Before use update DIR_MODEL and DIR_CONFIG
"""

from imageai.Detection.Custom import CustomObjectDetection
import cv2, time

DIR_MODEL = "detection_model-ex-58--loss-1.79.h5"
DIR_CONFIG = "detection_config.json"

# Find usb webcam
cap = None
cap_i = 0
for i in range(1, 11):
    try:
        print(i)
        cap = cv2.VideoCapture(i)
        _, frame = cap.read()
        cv2.imshow("aa", frame)
        cv2.destroyAllWindows()
        cap_i = i
        break
    except Exception:
        pass

detector = CustomObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setJsonPath(DIR_CONFIG)
detector.setModelPath(DIR_MODEL)
detector.loadModel()

# cap2 = VideoCapture2(cap_i)

start_t = time.time()
num = 1
while cv2.waitKey(1) != ord('q'):
    # Timming
    end_t = time.time()
    dt = end_t - start_t
    debug_str = "Frame: %2d   dt:%4f   FPS: %2.3f" % (num, dt, 1 / dt)
    print(debug_str)
    start_t = time.time()
    num += 1

    try:
        for i in range(7):  # Cleans buffer and reads current frame
            _, frame = cap.read()  # Get new frame from camera

        # detections = detector.detectObjectsFromImage(input_type="array", input_image=frame,
        #                                              output_image_path="image2new.jpg",
        #                                              minimum_percentage_probability=50)
        frame_out, detections = detector.detectObjectsFromImage(input_type="array", input_image=frame,
                                                                output_type="array", minimum_percentage_probability=1)
        for det in detections:
            print(det["name"], " : ", det["percentage_probability"], " : ", det["box_points"])
    except Exception:
        num -= 1
        print("Error in detection. Skipping frame")
        break

    # Debug text on image
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (5, 20)
    fontScale = 0.4
    color = (255, 0, 0)
    thickness = 1
    frame_out = cv2.putText(frame_out, debug_str, org, font,
                            fontScale, color, thickness, cv2.LINE_AA)

    cv2.imshow("Gold bag detector", frame_out)
    # cv2.imwrite("file%d.jpg" % num, frame_out)

cap.release()
cv2.destroyAllWindows()
