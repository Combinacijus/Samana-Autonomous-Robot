"""
    Gintaras Grebliunas
    combinacijus@gmail.com

    This script draws bounding box around gold bag and calculates
    center position of the box

    NOTE: Before use update DIR_MODEL and DIR_CONFIG
"""

from imageai.Detection.Custom import CustomObjectDetection
import cv2

DIR_MODEL = "detection_model-ex-58--loss-1.79.h5"
DIR_CONFIG = "detection_config.json"

# Find usb webcam
# cap = None
for i in range(1, 11):
    try:
        print(i)
        cap = cv2.VideoCapture(i)
        _, frame = cap.read()
        cv2.imshow("aa", frame)
        cv2.destroyAllWindows()
        break
    except Exception:
        pass


detector = CustomObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setJsonPath(DIR_CONFIG)
detector.setModelPath(DIR_MODEL)
detector.loadModel()

num = 1
while cv2.waitKey(1) != ord('q'):
    print("frame", num)
    num += 1
    _, frame = cap.read()
    # frame = cv2.resize(frame, (244, 157))
    # detections = detector.detectObjectsFromImage(input_type="array", input_image=frame,
    #                                              output_image_path="image2new.jpg",
    #                                              minimum_percentage_probability=50)

    print("before")
    frame_out, detections = detector.detectObjectsFromImage(input_type="array", input_image=frame,
                                                            output_type="array", minimum_percentage_probability=50)
    for eachObject in detections:
        print(eachObject["name"], " : ", eachObject["percentage_probability"], " : ", eachObject["box_points"])
    print("after")

    # font
    font = cv2.FONT_HERSHEY_SIMPLEX

    org = (50, 50)
    fontScale = 3
    color = (255, 0, 0)
    # Line thickness of 2 px
    thickness = 2
    frame = cv2.putText(frame, 'OpenCV', org, font,
                        fontScale, color, thickness, cv2.LINE_AA)

    cv2.imshow("Gold bag detector", frame_out)
    # cv2.imshow("Gold bag detector", frame)

cap.release()
cv2.destroyAllWindows()
