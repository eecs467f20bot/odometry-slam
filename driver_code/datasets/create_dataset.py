import cv2
import math
import time
import sys
import os

if len(sys.argv) != 3:
    print("Usage: python3 create_dataset.py {new_dataset_name} {video_input}")
    quit()

ds_name = sys.argv[1]
vid_filename = sys.argv[2]

video = cv2.VideoCapture(vid_filename)

if not os.path.exists(ds_name):
    os.makedirs(ds_name)
else:
    os.remove(ds_name + "/*")

framerate = 16.0
print("Framerate = {}".format(framerate))

timestamp_file = open("../timestamps/" + ds_name + ".txt", "w")

sim_time = 0

success, image = video.read()

while success:
    timestamp_file.write(str(math.floor(sim_time)) + "\n")
    cv2.imwrite(ds_name + "/" + str(math.floor(sim_time)) + ".png", image)
    success, image = video.read()

    sim_time += (1.0/framerate) * 1e9

timestamp_file.close()
