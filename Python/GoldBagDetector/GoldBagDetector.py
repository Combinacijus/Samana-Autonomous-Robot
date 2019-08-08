TODO: DETECTION CONFIG HAVE A BUG SEE IMAGE AI GITHUB

"""
    Gintaras Grebliunas
    combinacijus@gmail.com

    This script draws bounding box around gold bag and calculates
    center position of the box

    NOTE: Before use update DIR_MODEL and DIR_CONFIG
"""

from imageai.Detection.Custom import CustomObjectDetection
import cv2

DIR_MODEL = "detection_model-ex-104--loss-2.93.h5"
DIR_CONFIG = "detection_config.json"

detector = CustomObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setJsonPath(DIR_CONFIG)
detector.setModelPath(DIR_MODEL)
detector.loadModel()

cap = cv2.VideoCapture(0)

while cv2.waitKey(1) != ord('q'):
    _, frame = cap.read()
    frame = cv2.resize(frame, (244, 157))
    # detections = detector.detectObjectsFromImage(input_type="array", input_image=frame,
    #                                              output_image_path="image2new.jpg",
    #                                              minimum_percentage_probability=30)
    frame_out, detections = detector.detectObjectsFromImage(input_type="array", input_image=frame,
                                                            output_type="array", minimum_percentage_probability=30)
    for eachObject in detections:
        print(eachObject["name"], " : ", eachObject["percentage_probability"], " : ", eachObject["box_points"])

    cv2.imshow("Gold bag detector", frame_out)

cap.release()
cv2.destroyAllWindows()
