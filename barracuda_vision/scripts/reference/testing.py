#!/usr/bin/env python3
import rospy
from time import sleep
from enum import Enum
import math
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from barracuda_vision.msg import ballInfo, ballTravel
from dynamic_reconfigure.server import Server
import cv2
from filtercam import filter_cam 
import shapegui as sp
import numpy as np
from threading import Thread
from queue import Queue

#--PUBLISHER--#
ballStatus_pub = rospy.Publisher("ball/ballStatus", Bool, queue_size= 1)
ballPoint_pub = rospy.Publisher("ball/ballPoint", ballInfo, queue_size= 10)
ballPos_pub = rospy.Publisher("ball/ballPos", ballTravel, queue_size= 10)
#--MESSAGE--#
ball_point = ballInfo()
ball_point.x = 0
ball_point.y = 0
ball_point.radius = 0

ball_pos = ballTravel()
ball_pos.distance = 0
ball_pos.degree = 0
#--CALC FUNCTION--#
def getAngle(CF_x, CF_y, X_ball, Y_ball):
    dx = X_ball - CF_x
    dy = CF_y - Y_ball

    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)

    if angle_deg < 0:
        angle_deg += 360

    return int(angle_deg) 

def getDistance(focal_length, real_size, rad_2x):
    return (focal_length * real_size) / rad_2x



def preprocess_frame(frame):
    """
    Preprocess the frame for adaptive lighting using CLAHE.
    """
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Apply CLAHE to the V channel for better lighting adaptation
    h, s, v = cv2.split(hsv)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    v = clahe.apply(v)
    
    # Merge the adjusted V channel back to HSV
    hsv = cv2.merge((h, s, v))
    
    return hsv

def detect_ball(frame, lower_orange, upper_orange):
    """
    Detect an orange ball in the frame using color-based segmentation.
    Returns the ball's center (x, y), radius, and angle (if applicable).
    """
    # Preprocess the frame for adaptive lighting
    hsv = preprocess_frame(frame)

    # Create a mask for the orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour (assumed to be the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

        # Calculate the angle (for ballStraightPoint)
        if radius > 7.43:  # Only process if the ball is large enough
            ellipse = cv2.fitEllipse(largest_contour)
            angle = ellipse[2]  # Angle of the major axis
            return (int(x), int(y)), int(radius), angle, mask  # Return mask as well
    return None, None, None, mask  # Return mask even if no ball is detected

def init_kalman():
    """
    Initialize the Kalman Filter.
    """
    kalman = cv2.KalmanFilter(4, 2)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
    return kalman

def track_ball(kalman, x, y):
    """
    Track the ball using Kalman Filter.
    """
    measurement = np.array([[np.float32(x)], [np.float32(y)]])
    kalman.correct(measurement)
    prediction = kalman.predict()
    return int(prediction[0]), int(prediction[1])

def capture_frames(cap, frame_queue):
    """
    Thread function to continuously capture frames from the camera.
    """
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame!")
            continue
        if frame_queue.full():
            frame_queue.get()  # Remove the oldest frame if the queue is full
        frame_queue.put(frame)
#--GLOBAL VARIABEL--#
ball_size = 9.25 #ukuran dalam cm
# lowerHSV_ball = np.array([Hue_min,Sat_min,Val_min])
# upperHSV_ball = np.array([Hue_max,Sat_max,Val_max])
font = cv2.FONT_HERSHEY_SIMPLEX
fontSize = 1.5
thickFont = 2
#--CLASS VIDEO (PENGATURAN VIDEO)--#
class VideoStream:
    def __init__(self, src=0, exposure_val=0.0, width=640, height=480):
        self.stream = cv2.VideoCapture(src)

        self.stream.set(cv2.CAP_PROP_FPS, 30)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.frame_width = int(self.stream.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rospy.loginfo((self.frame_width, self.frame_height))
        self.fps = self.stream.get(cv2.CAP_PROP_FPS)
        rospy.loginfo(self.fps)

        if self.stream.get(cv2.CAP_PROP_EXPOSURE) is not None:
            self.exposure_val = self.stream.get(cv2.CAP_PROP_EXPOSURE)
            self.exposure_val = exposure_val
            self.stream.set(cv2.CAP_PROP_EXPOSURE, self.exposure_val)
        if self.stream.get(cv2.CAP_PROP_AUTOFOCUS) is not None:
            self.stream.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

#--MAIN FUNCTION --#
if __name__ == '__main__':
    rospy.init_node('omni_cam')

    cam_index = rospy.get_param("~cam_index", 0)
    exposure_val = rospy.get_param("~exposure_val", 1)
    vs = VideoStream(src=cam_index, exposure_val=exposure_val).start()

    frame_width = vs.frame_width
    frame_height = vs.frame_height

    flip_frame = rospy.get_param("~flip_frame", False)
    # flip_code = rospy.get_param("~flip_code", 1)

    blur_val = rospy.get_param("~blur_val", 1)
    offset_W = rospy.get_param("~offset_W", 0)
    offset_H = rospy.get_param("~offset_H", 0)
    mode = rospy.get_param("~modeContour", 1)
    focal_lenght = rospy.get_param("~focalCam", 217)
    cam_center = (frame_width // 2 + offset_W, frame_height // 2 - offset_H)

    Hue_min = rospy.get_param("~Hue_min", 5)
    Hue_max = rospy.get_param("~Hue_max", 52)
    Sat_min = rospy.get_param("~Sat_min", 154)
    Sat_max = rospy.get_param("~Sat_max", 255)
    Val_min = rospy.get_param("~Val_min", 93)
    Val_max = rospy.get_param("~Val_max", 255)

    line_detect = rospy.get_param("~line_detect", 0)
    Hue_min_line = rospy.get_param("~Hue_min_line", 0)
    Hue_max_line = rospy.get_param("~Hue_max_line", 0)
    Sat_min_line = rospy.get_param("~Sat_min_line", 0)
    Sat_max_line = rospy.get_param("~Sat_max_line", 0)
    Val_min_line = rospy.get_param("~Val_min_line", 0)
    Val_max_line = rospy.get_param("~Val_max_line", 0)

    slopeToggle = rospy.get_param("~slopeToggle", 0)
    line_v1 = rospy.get_param("~line_v1", 0)
    line_v2 = rospy.get_param("~line_v2", 0)
    length_v = rospy.get_param("~length_v", 0)
    line_h1 = rospy.get_param("~line_h1", 0)
    line_h2 = rospy.get_param("~line_h2", 0)
    length_h = rospy.get_param("~length_h", 0)
    slopeLine = rospy.get_param("~slopeLine", 0)

    dummyToggle = rospy.get_param("~dummyToggle", 0)
    Hue_min_dummy = rospy.get_param("~Hue_min_dummy", 0)
    Hue_max_dummy = rospy.get_param("~Hue_max_dummy", 0)
    Sat_min_dummy = rospy.get_param("~Sat_min_dummy", 0)
    Sat_max_dummy = rospy.get_param("~Sat_max_dummy", 0)
    Val_min_dummy = rospy.get_param("~Val_min_dummy", 0)
    Val_max_dummy = rospy.get_param("~Val_max_dummy", 0)

    crop_side_dummy = rospy.get_param("~crop_side_dummy", 0)
    crop_top_dummy = rospy.get_param("~crop_top_dummy", 0)
    crop_bottom_dummy = rospy.get_param("~crop_bottom_dummy", 0)

    centerF_x = frame_width//2
    centerF_y = frame_height//2
    
    rate = rospy.Rate(50)
    try:
        while not rospy.is_shutdown():
            frame = vs.read()
            frame = cv2.flip(frame, 1)
            if not vs.grabbed: break

            # if flip_frame:
            #     frame = filter_cam.flip_frame(frame, flip_code)
            frame_ui = frame.copy()
            sp.crossPlus(frame_ui)
            if mode == 0:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                masking = cv2.inRange(hsv,(Hue_min, Sat_min, Val_min),(Hue_max, Sat_max, Val_max))
                contour, hierarcy = cv2.findContours(masking, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contour:
                    largest_contour = max(contour, key=cv2.contourArea)
                    ((x,y), radius) = cv2.minEnclosingCircle(largest_contour)
                    if radius > 10:
                        ballStatus_pub.publish(1)
                        ball_point.x = int(x)
                        ball_point.y = int(y)
                        ball_point.radius = int(radius)
                        ball_pos.distance = int(getDistance(focal_lenght, ball_size, ball_point.radius * 2))
                        ball_pos.degree = getAngle(centerF_x, centerF_y , ball_point.x, ball_point.y)
                        ballPoint_pub.publish(ball_point)
                        ballPos_pub.publish(ball_pos)
                        sp.circle(frame_ui, ball_point.x, ball_point.y, ball_point.radius, "HIJAU", 1)
                        sp.line(frame_ui,centerF_x, centerF_y, ball_point.x, ball_point.y,"BIRU", 2)
                        sp.text(frame_ui, f"x:{ball_point.x}, y:{ball_point.y},rad:{ball_point.radius}", 20, 40,"SIMPLEX", 0.5,"HIJAU", 1 )
                        sp.text(frame_ui, f"distance:{ball_pos.distance}, degree:{ball_pos.degree}", 20, 70,"SIMPLEX", 0.5,"HIJAU", 1 )
                        sp.text(frame_ui, f"BALL", 600, 40,"SIMPLEX", 0.5,"HIJAU", 2 )
                        # rospy.loginfo(f"position, xValue:{x}, yValue:{y}, radValue{radius}")
                else :
                    ballStatus_pub.publish(0)
                    sp.text(frame_ui, f"BALL", 600, 40,"SIMPLEX", 0.5,"MERAH", 2 )
                    sp.text(frame_ui, f"x:NONE, y:NONE,rad:NONE", 20, 40,"SIMPLEX", 0.5,"MERAH", 1 )
                    sp.text(frame_ui, f"distance:NONE, degree:NONE", 20, 70,"SIMPLEX", 0.5,"MERAH", 1 )
            elif mode == 1:
                kalman = init_kalman()
                hsv = preprocess_frame(frame)
                masking = cv2.inRange(hsv, (Hue_min, Sat_min, Val_min), (Hue_max, Sat_max, Val_max))
                contour, hierarchy = cv2.findContours(masking, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contour:
                    # Temukan kontur terbesar (bola)
                    largest_contour = max(contour, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

                    if radius > 10:  # Hanya proses jika radius bola cukup besar
                        # Lacak bola menggunakan Kalman Filter
                        predicted_x, predicted_y = track_ball(kalman, int(x), int(y))

                        # Publikasikan status bola
                        ballStatus_pub.publish(1)
                        ball_point.x = int(x)
                        ball_point.y = int(y)
                        ball_point.radius = int(radius)
                        ball_pos.distance = int(getDistance(focal_lenght, ball_size, ball_point.radius * 2))
                        ball_pos.degree = getAngle(centerF_x, centerF_y, ball_point.x, ball_point.y)
                        ballPoint_pub.publish(ball_point)
                        ballPos_pub.publish(ball_pos)

                        # Gambar hasil deteksi dan prediksi di frame_ui
                        sp.circle(frame_ui, ball_point.x, ball_point.y, ball_point.radius, "HIJAU", 1)
                        sp.line(frame_ui, centerF_x, centerF_y, ball_point.x, ball_point.y, "BIRU", 2)
                        sp.text(frame_ui, f"x:{ball_point.x}, y:{ball_point.y}, rad:{ball_point.radius}", 20, 40, "SIMPLEX", 0.5, "HIJAU", 1)
                        sp.text(frame_ui, f"distance:{ball_pos.distance}, degree:{ball_pos.degree}", 20, 70, "SIMPLEX", 0.5, "HIJAU", 1)
                        sp.text(frame_ui, f"BALL", 600, 40, "SIMPLEX", 0.5, "HIJAU", 2)

                        # Gambar posisi prediksi dari Kalman Filter
                        cv2.circle(frame_ui, (predicted_x, predicted_y), 5, (0, 0, 255), -1)  # Titik merah untuk prediksi
                else:
                    # Jika tidak ada bola yang terdeteksi
                    ballStatus_pub.publish(0)
                    sp.text(frame_ui, f"BALL", 600, 40, "SIMPLEX", 0.5, "MERAH", 2)
                    sp.text(frame_ui, f"x:NONE, y:NONE, rad:NONE", 20, 40, "SIMPLEX", 0.5, "MERAH", 1)
                    sp.text(frame_ui, f"distance:NONE, degree:NONE", 20, 70, "SIMPLEX", 0.5, "MERAH", 1)
            # mask = cv2.inRange(hsv, (Hue_min, Sat_min, Val_min), (Hue_max, Sat_max, Val_max))
            masked_frame = cv2.bitwise_and(frame, frame, mask=masking)

            cv2.imshow("Original Frame1", frame)
            cv2.imshow("HSV Frame", hsv)
            cv2.imshow("Mask", masking)
            cv2.imshow("Masked Frame", masked_frame)
            cv2.imshow("Frame UI", frame_ui)
 
            
            cv2.waitKey(1)

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")
    finally:
        vs.stop()
        cv2.destroyAllWindows()