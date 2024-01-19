#!/usr/bin/python3

import cv2
import numpy as np
from time import sleep
from picamera2 import Picamera2

# Grab images as numpy arrays and leave everything else to OpenCV.

face_detector = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
cv2.startWindowThread()

width = 1280
height = 720

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (width, height)}))
picam2.start()

# Give the camera a good long time to set gains and
# measure AWB (you may wish to use fixed AWB instead)
sleep(5)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((44, 3), np.float32)
spacing = 0.787402 # inches (2cm)
objp[0]  = (spacing*0   , spacing*0  , 0)
objp[1]  = (spacing*0   , spacing*1 , 0)
objp[2]  = (spacing*0   , spacing*2, 0)
objp[3]  = (spacing*0   , spacing*3, 0)
objp[4]  = (spacing*0.5 , spacing*0.5 , 0)
objp[5]  = (spacing*0.5 , spacing*1.5, 0)
objp[6]  = (spacing*0.5 , spacing*2.5, 0)
objp[7]  = (spacing*0.5 , spacing*3.5, 0)
objp[8]  = (spacing*1   , spacing*0  , 0)
objp[9]  = (spacing*1   , spacing*1 , 0)
objp[10] = (spacing*1   , spacing*2, 0)
objp[11] = (spacing*1   , spacing*3, 0)
objp[12] = (spacing*1.5 , spacing*0.5,  0)
objp[13] = (spacing*1.5 , spacing*1.5, 0)
objp[14] = (spacing*1.5 , spacing*2.5, 0)
objp[15] = (spacing*1.5 , spacing*3.5, 0)
objp[16] = (spacing*2   , spacing*0  , 0)
objp[17] = (spacing*2   , spacing*1 , 0)
objp[18] = (spacing*2   , spacing*2, 0)
objp[19] = (spacing*2   , spacing*3, 0)
objp[20] = (spacing*2.5 , spacing*0.5 , 0)
objp[21] = (spacing*2.5 , spacing*1.5, 0)
objp[22] = (spacing*2.5 , spacing*2.5, 0)
objp[23] = (spacing*2.5 , spacing*3.5, 0)
objp[24] = (spacing*3, spacing*0  , 0)
objp[25] = (spacing*3, spacing*1 , 0)
objp[26] = (spacing*3, spacing*2, 0)
objp[27] = (spacing*3, spacing*3, 0)
objp[28] = (spacing*3.5, spacing*0.5 , 0)
objp[29] = (spacing*3.5, spacing*1.5, 0)
objp[30] = (spacing*3.5, spacing*2.5, 0)
objp[31] = (spacing*3.5, spacing*3.5, 0)
objp[32] = (spacing*4, spacing*0  , 0)
objp[33] = (spacing*4, spacing*1 , 0)
objp[34] = (spacing*4, spacing*2, 0)
objp[35] = (spacing*4, spacing*3, 0)
objp[36] = (spacing*4.5, spacing*0.5 , 0)
objp[37] = (spacing*4.5, spacing*1.5, 0)
objp[38] = (spacing*4.5, spacing*2.5, 0)
objp[39] = (spacing*4.5, spacing*3.5, 0)
objp[40] = (spacing*5, spacing*0  , 0)
objp[41] = (spacing*5, spacing*1 , 0)
objp[42] = (spacing*5, spacing*2, 0)
objp[43] = (spacing*5, spacing*3, 0)
#########################################


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
count = 0

while count <50:
    img = picam2.capture_array()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    #ret, corners = cv2.findChessboardCorners(gray, (8,5), None)
    ret, corners = cv2.findCirclesGrid(gray, (4, 11), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
    # If found, add object points, image points (after refining them)
    if ret == True:
        count = count + 1
        objpoints.append(objp)
        #corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (4,11), corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
    else:
        cv2.imshow('img', img)
        cv2.waitKey(1)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("Camera matrix : \n")
print(mtx)
print("dist : \n")
print(dist)
print("rvecs : \n")
print(rvecs)
print("tvecs : \n")
print(tvecs)


data = {"camera_matrix": mtx.tolist(), "dist_coeff": dist.tolist(), "width": width, "height":height}
fname = "cam_cal_"+str(width)+"_"+str(height)+".json"
import json
with open(fname, "w") as f:
    json.dump(data, f)


print("now get ready, camera is switching on")
while(1):
    image = picam2.capture_array()
    h,  w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    # undistort
    dst = cv2.undistort(image, mtx, dist, None, newcameramtx)

    cv2.imshow('dst', dst)
    cv2.imshow('img', image)
    cv2.waitKey(1)
		
print("everything is fine")