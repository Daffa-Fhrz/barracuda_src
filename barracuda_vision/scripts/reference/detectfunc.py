import cv2
from filtercam import filter_cam
import numpy as np
import rospy
import math

ft = filter_cam()

def adaptiveLight(frameSet, clipLimit = 2.0, tileGrid = (8,8)):
    hsv = ft.hsvFilter(frameSet)
    h, s, v = cv2.split(hsv)
    clahe = cv2.createCLAHE(clipLimit,tileGrid)
    hsv = cv2.merge((h, s, v))
    return hsv

def detect_ballEC(frameSet, H_min = 5, S_min = 150, V_min = 100, H_max = 15, S_max = 255, V_max = 255, rad_value = 7.43):
    mask = ft.mask(frameSet, H_min, S_min, V_min, H_max, S_max, V_max)
    contours, _ = ft.contourDetect(mask,0,1)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(largest_contour)

        if radius > rad_value:
            return (int(x), int(y)), int(radius), mask
    return (0, 0), 0, mask

def detect_ballBR(frameSet,cam_center, H_min = 5, S_min = 150, V_min = 100, H_max = 15, S_max = 255, V_max = 255, rad_value = 7.43):
    mask = ft.mask(frameSet, H_min, S_min, V_min, H_max, S_max, V_max)
    contours, _ = ft.contourDetect(mask,0,1)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        centroid = (x + w // 2, y + h // 2)
        ball_radius = max(w, h) // 2
        if ball_radius > rad_value:
            x = (centroid[0] - cam_center[0]) / cam_center[0]
            y = (cam_center[1] - centroid[1]) / cam_center[1]
            gradien = math.sqrt(x ** 2 + y ** 2)
            degree = math.degrees(math.atan2(x, y))
            return (x,y), gradien, degree, mask
            
    return (0, 0), 0, 0, mask

def init_kalman():
    kalman = cv2.KalmanFilter(4, 2)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
    return kalman

def trackBall(kalman, x, y):
    measurement = np.array([[np.float32(x)], [np.float32(y)]])
    kalman.correct(measurement)
    prediction = kalman.predict()
    return int(prediction[0]), int(prediction[1])

def queueFrame(cap, frame_queue):
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Gagal mengambil gambar")
            continue
        if frame_queue.full():
            frame_queue.get()
        frame_queue.put(frame)

def distanceCalc(focal_length, real_size, rad_2x):
    return (focal_length * real_size) / rad_2x

def angleCalc(centerX_frame, centerY_frame, x_ball, y_ball):
    dx = x_ball - centerX_frame
    dy = centerY_frame - y_ball

    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)

    if angle_deg < 0:
        angle_deg += 360

    return int(angle_deg)