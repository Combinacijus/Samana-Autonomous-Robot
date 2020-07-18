"""
    Separates annotated data set to validation data set

    Selects random files with name "IMG000.jpeg" and "IMG000.xml" from specified directories
    And moves to validation set directory

    Read NOTE: comments
"""

import shutil, random, os
from parse import parse

BASE_DIR = "/home/combinacijus/Documents/SamanaAutonomousRobot/Python/GoldBagDetector/training-images/"  # NOTE: change base dir
FROM_DIR = BASE_DIR + "dataset1/train/"  # NOTE: change to training set dir
TO_DIR = BASE_DIR + "dataset1/validation/"  # NOTE: change to validation set dir (will be created new)
VALIDATION_FRACTION = 0.2  # NOTE: fraction of images to pick for validation

PARSE_IMG = "{}.jpg"  # NOTE: change for different file naming
PARSE_ANOT = "{}.xml"  # NOTE: change for different file naming

IMG_DIR = "images/"
ANOT_DIR = "annotations/"

img_list = os.listdir(FROM_DIR + IMG_DIR)

# Groups images with its annotations
anot = []
for img in img_list:
    name = parse(PARSE_IMG, img)[0]
    anot.append(PARSE_ANOT.format(name))

img_count = len(img_list)
validation_count = int(img_count * VALIDATION_FRACTION)
print("Images      : {}".format(img_list))
print("Annotations : {}".format(anot))
print("Total images: {}".format(img_count))
print("Images and annotations to be moved (displayed only 5):")

files = list(zip(img_list, anot))
validation_list = random.sample(files, validation_count)
for v in validation_list[:5]:
    print(v)
print("\nNumber of selected images: {}".format(validation_count))

print("")
print("From: {}".format(FROM_DIR))
print("TO   : {}".format(TO_DIR))
answer = input("Move shown items? y/n: ")

if answer == "y":
    for (img, anot) in validation_list:
        from_file_img = FROM_DIR + IMG_DIR + img
        from_file_anot = FROM_DIR + ANOT_DIR + anot
        to_dir_img = TO_DIR + IMG_DIR
        to_dir_anot = TO_DIR + ANOT_DIR
        to_file_img = to_dir_img + img
        to_file_anot = to_dir_anot + anot

        if not os.path.exists(to_dir_img):
            os.makedirs(to_dir_img)
        if not os.path.exists(to_dir_anot):
            os.makedirs(to_dir_anot)

        print("Moved: {}  {}".format(from_file_img, to_file_img))
        print("Moved: {}  {}".format(from_file_anot, to_file_anot))
        shutil.move(from_file_img, to_file_img)
        shutil.move(from_file_anot, to_file_anot)

print("Program finished")
