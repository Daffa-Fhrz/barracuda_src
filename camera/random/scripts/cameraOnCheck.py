
import cv2
import numpy as np
import os
import glob
 
CHECKERBOARD = (7,10)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = [] 
 
# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

source =  cv2.VideoCapture(1)
alive = True
winName = 'Tester'
cv2.namedWindow(winName, cv2.WINDOW_NORMAL)

while alive:
    frameOn, frame = source.read()
    if not frameOn:
        break
    frame = cv2.flip(frame, 1)
    frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(frameGray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(frameGray, corners, (11,11),(-1,-1), criteria)
        imgpoints.append(corners2)
        frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)

    cv2.imshow(winName,frame)
    key = cv2.waitKey(1)
    if key == ord("Q") or key == ord("q") or key == 27:
        alive = False
 
source.release()
cv2.destroyWindow(winName)
 
