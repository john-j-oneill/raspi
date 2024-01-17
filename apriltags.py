#!/usr/bin/python3

import cv2
import numpy as np
from time import sleep
#import apriltag
from dt_apriltags import Detector
from picamera2 import Picamera2
import json

width = 640
height = 480

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
                       nthreads=3,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


print("now get ready, camera is switching on")
while(1):
    image = picam2.capture_array()
    h,  w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    # undistort
    dst = cv2.undistort(image, mtx, dist, None, newcameramtx)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # define the AprilTags detector options and then detect the AprilTags
    # in the input image
    #results = detector.detect(gray)

    tag_size = 6 # Inches?
    cameraMatrix = mtx
    camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )
    tags = at_detector.detect(gray, True, camera_params, tag_size)
    #print(tags)
    print("[INFO] {} total AprilTags detected".format(len(tags)))

    for tag in tags:
        print(tag)
        for idx in range(len(tag.corners)):
            cv2.line(image, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

        cv2.putText(image, str(tag.tag_id),
                    org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255))

    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imshow('dst', dst)
    cv2.imshow('img', image)
    k = cv2.waitKey(30) & 0xff
    if k == 27: # press 'ESC' to quit
        break
		
print("everything is fine")