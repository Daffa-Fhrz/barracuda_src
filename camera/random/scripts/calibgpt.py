import cv2
import numpy as np
import os
import glob
 
CHECKERBOARD = (7, 10)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = []
 
# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Extracting path of individual image stored in a given directory
images = glob.glob('cb2/WIN_20241223_23_11_56_Pro.jpg')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    # Refine corners if detected
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Checkerboard Corners', img)
        cv2.waitKey(0)
 
cv2.destroyAllWindows()
 
h, w = img.shape[:2]
# Perform calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)
print("Calibration Results:")
print(f"Camera Matrix:\n{mtx}")
print(f"Distortion Coefficients:\n{dist}")

# Initialize omnidirectional rectification
xi = 0.5  # Assumes a generic value; calibrate for accurate results
size = (640, 480)  # Desired output resolution

# Compute rectification map
flags = cv2.omnidir.RECTIFY_PERSPECTIVE  # Change flag as needed
map1, map2 = cv2.omnidir.initUndistortRectifyMap(
    mtx, dist, xi, None, mtx, size, cv2.CV_16SC2, flags)

# Remap the original image
image = cv2.imread(images[0])  # Test with the first image
rectified_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)

# Display rectified image
cv2.imshow("Rectified Image", rectified_image)
cv2.imwrite("rectified_image.jpg", rectified_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
