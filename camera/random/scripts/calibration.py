import cv2
import numpy as np
import os
import glob
 
CHECKERBOARD = (6,9)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 70, 0.001)
 
# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = [] 

objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

img = cv2.imread('stereo/WIN_20241222_09_12_44_Pro.jpg', cv2.IMREAD_COLOR)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

retval, corner = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_ADAPTIVE_THRESH  + cv2.CALIB_CB_NORMALIZE_IMAGE )

objpoints.append(objp)
corners2 = cv2.cornerSubPix(gray, corner, (11,11),(-1,-1), criteria) 
imgpoints.append(corners2)
img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, retval)


cv2.imshow('display', img)
cv2.waitKey(0)