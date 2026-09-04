#!/usr/bin/env python3
import rospy
import cv2
import math  # Import the math module
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from barracuda_vision.msg import ballInfo, ballTravel
from cv_bridge import CvBridge
import numpy as np
import shapegui as sp

mode = rospy.get_param('mode_hsv', 0)
H_min = rospy.get_param('Hue_min', 0)
H_max = rospy.get_param('Hue_max', 15)
S_min = rospy.get_param('Sat_min', 150)
S_max = rospy.get_param('Sat_max', 255)
V_min = rospy.get_param('Val_min', 100)
V_max = rospy.get_param('Val_max', 255)
rad_ball = rospy.get_param('rad_ball', 8)
focalCam = rospy.get_param('focal_lenght', 2400)


ball_size = 21.0  # 21 cm ball size
centerF_x = 320  # Example value, replace with actual value
centerF_y = 240  # Example value, replace with actual value

ball_point = ballInfo()
ball_point.x = 0
ball_point.y = 0
ball_point.radius = 0

ball_pos = ballTravel()
ball_pos.distance = 0
ball_pos.degree = 0

x1roi = 180
x2roi = 440
y1roi = 90
y2roi = 390

def getAngle(CF_x, CF_y, X_ball, Y_ball):
    dx = X_ball - CF_x
    dy = CF_y - Y_ball

    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)

    if angle_deg < 0:
        angle_deg += 360

    return int(angle_deg)

def getDistance(focal_length, real_size, rad_2x):
    if rad_2x <= 0:
        return 0
    distance = (focal_length * real_size) / rad_2x
    return max(0, round(distance))  # Round properly and ensure non-negative

class hsvball_omni:
    global mode
    def __init__(self):
        rospy.init_node('hsvOmni_ball', anonymous=False)
        
        self.bridge = CvBridge()
        self.image_get = rospy.Subscriber('/barracuda_vision/camera/omni_raw', Image, self.hsv_callback)
        self.ballInfo_pub = rospy.Publisher('/barracuda_vision/camera/omni/ballInfo', ballInfo, queue_size=10)
        self.ballTravel_pub = rospy.Publisher('/barracuda_vision/camera/omni/ballTravel', ballTravel, queue_size=10)
        self.ballStatus_pub = rospy.Publisher("/barracuda_vision/camera/omni/ballStatus", Bool, queue_size=1)
        self.hsvOmni_pub = rospy.Publisher('/barracuda_vision/camera/omni_hsv_mask', Image, queue_size=10)
        self.hsvMergeOmni_pub = rospy.Publisher('/barracuda_vision/camera/omni_hsv_merge', Image, queue_size=10)

    def hsv_callback(self, msg):
        global mode
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if cv_image is None:
            rospy.logwarn("Frame tidak diterima")

        if mode == 0:

            cv_image = cv2.GaussianBlur(cv_image, (15,15), 0)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Buat mask berdasarkan range HSV
            lower_hsv = np.array([H_min, S_min, V_min])
            upper_hsv = np.array([H_max, S_max, V_max])

            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            kernel = np.ones((15, 15), np.uint8)
            mask_dilatasi = cv2.dilate(mask, kernel, iterations=1)
            roi = mask_dilatasi[y1roi:y2roi, x1roi:x2roi]
            mask_erode = cv2.erode(roi, kernel, iterations=1)
            mask_dilatasi[y1roi:y2roi, x1roi:x2roi] = mask_erode
            mask = mask_dilatasi

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.hsvOmni_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
                if radius > rad_ball:
                    self.ballStatus_pub.publish(1)
                    ball_point.x = int(x)
                    ball_point.y = int(y)
                    ball_point.radius = int(radius)
                    ball_pos.distance = getDistance(focalCam, ball_size, ball_point.radius * 2)
                    ball_pos.degree = getAngle(centerF_x, centerF_y, ball_point.x, ball_point.y)
                    self.ballInfo_pub.publish(ball_point)
                    self.ballTravel_pub.publish(ball_pos)
                else:
                    self.ballStatus_pub.publish(0)
                    ball_point.x = 0
                    ball_point.y = 0
                    ball_point.radius = 0
                    ball_pos.distance = 0
                    ball_pos.degree = 0
                    self.ballInfo_pub.publish(ball_point)
                    self.ballTravel_pub.publish(ball_pos)
            else:
                self.ballStatus_pub.publish(0)
                ball_point.x = 0
                ball_point.y = 0
                ball_point.radius = 0
                ball_pos.distance = 0
                ball_pos.degree = 0
                self.ballInfo_pub.publish(ball_point)
                self.ballTravel_pub.publish(ball_pos)
            masked_frame = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            self.hsvMergeOmni_pub.publish(self.bridge.cv2_to_imgmsg(masked_frame, "bgr8"))
        elif mode == 1:
            pass
        elif mode == 2:
            pass
        else:
            mode = 0

if __name__ == "__main__":
    try:
        omniRun = hsvball_omni()
        rospy.spin()
    except rospy.ROSInterruptException:  # Corrected exception name
        pass
