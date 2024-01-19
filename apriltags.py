#!/usr/bin/python3

import cv2
import numpy as np
from time import sleep
#import apriltag
from dt_apriltags import Detector
from picamera2 import Picamera2
from scipy.spatial.transform import Rotation
import json
import can
import time


width = 1280
height = 720
undistort = False
display = False

# Opening JSON file
fname = "cam_cal_"+str(width)+"_"+str(height)+".json"
f = open(fname)

# returns JSON object as
# a dictionary
data = json.load(f)

# Iterating through the json
# list

mtx = np.array(data['camera_matrix'])
mtx = mtx.reshape(3, 3)
print(mtx)
dist = np.array(data['dist_coeff'])
print(dist)

# Closing file
f.close()

# Grab images as numpy arrays and leave everything else to OpenCV.

cv2.startWindowThread()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (width, height)}))
picam2.start()

# Give the camera a good long time to set gains and
# measure AWB (you may wish to use fixed AWB instead)
sleep(2)


#data = {"camera_matrix": mtx.tolist(), "dist_coeff": dist.tolist()}
#fname = "data.json"
#import json
#with open(fname, "w") as f:
#    json.dump(data, f)


print("[INFO] detecting AprilTags...")
#detector = apriltag("tag36h11")
#options = apriltag.DetectorOptions(families="tag36h11")
#detector = apriltag.Detector(options)

at_detector = Detector(families='tag36h11',
                       nthreads=4,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
frame_count = 0
fps_count = 0
start_time = time.time()
fps_interval = 5 # displays the frame rate every 5 second
fps = 0

print("[INFO] now get ready, camera is switching on")
while(1):
    image = picam2.capture_array()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # define the AprilTags detector options and then detect the AprilTags
    # in the input image
    #results = detector.detect(gray)

    tag_size = 6 # Inches?
    cameraMatrix = mtx
    camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )
    tags = at_detector.detect(gray, True, camera_params, tag_size)
    #print(tags)

    fps_count = fps_count + 1
    if (time.time() - start_time) > fps_interval :
        fps = fps_count / (time.time() - start_time)
        print("FPS: ", fps)
        fps_count = 0
        start_time = time.time()

    if display:
        print("[INFO] {} total AprilTags detected".format(len(tags)))

    with can.interface.Bus(channel = 'can0', bustype = 'socketcan') as can0:

        byte0 = frame_count & 0xFF
        byte1 = (frame_count >> 8) & 0xFF
        byte2 = (frame_count >> 16) & 0xFF
        byte3 = (frame_count >> 24) & 0xFF
        byte4 = abs(round(fps*100)) & 0xFF
        byte5 = (abs(round(fps*100)) >> 8) & 0xFF
        byte6 = len(tags) & 0xFF
        byte7 = 0
        msg = can.Message(is_extended_id=True, arbitration_id = 0xFFCF, data=[byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7])
        can0.send(msg)
    arb_id=0xFFC0
    for tag in tags:
        if display:
            print(tag)
            print(rot)
        rot = Rotation.from_matrix(tag.pose_R).as_euler(
                'zyx', degrees=False)

        for idx in range(len(tag.corners)):
            cv2.line(image, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

        cv2.putText(image, str(tag.tag_id),
                    org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255))

        with can.interface.Bus(channel = 'can0', bustype = 'socketcan') as can0:
            x_mm = round(tag.pose_t[0, 0] * 25.4)
            y_mm = round(tag.pose_t[2, 0] * 25.4)
            yaw_rad = rot[1]
            yaw_cdeg = round(yaw_rad * 18000 / 3.14159265358)
            byte0 = tag.tag_id & 0xFF
            byte1 = (tag.tag_id >> 8) & 0x03 | ((y_mm<0) << 5) | ((x_mm<0) << 6) | ((yaw_cdeg<0) << 7)
            byte2 = abs(yaw_cdeg) & 0xFF
            byte3 = (abs(yaw_cdeg) >> 8) & 0xFF
            byte4 = abs(x_mm) & 0xFF
            byte5 = (abs(x_mm) >> 8) & 0xFF
            byte6 = abs(y_mm) & 0xFF
            byte7 = (abs(y_mm) >> 8) & 0xFF
            msg = can.Message(is_extended_id=True, arbitration_id = arb_id, data=[byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7])
            can0.send(msg)
        arb_id = arb_id + 1

    if undistort:
        h,  w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        # undistort
        dst = cv2.undistort(image, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv2.imshow('dst', dst)
    if display:
        cv2.imshow('img', image)
        k = cv2.waitKey(1) & 0xff
        if k == 27: # press 'ESC' to quit
            break

    frame_count = frame_count + 1

print("everything is fine")