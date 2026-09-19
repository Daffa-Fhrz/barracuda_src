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
H_min = rospy.get_param('Hue_min', 7)
H_max = rospy.get_param('Hue_max', 15)
S_min = rospy.get_param('Sat_min', 150)
S_max = rospy.get_param('Sat_max', 255)
V_min = rospy.get_param('Val_min', 150)
V_max = rospy.get_param('Val_max', 255)
rad_ball = rospy.get_param('rad_ball', 8)
focalCam = rospy.get_param('focal_lenght', 5000)


ball_size = 1.0  # vm size
centerF_x = 320  # Example value, replace with actual value
centerF_y = 240  # Example value, replace with actual value

ball_point = ballInfo()
ball_point.x = 0
ball_point.y = 0
ball_point.radius = 0

ball_pos = ballTravel()
ball_pos.distance = 0
ball_pos.degree = 0

adjDistance = 10

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

class hsvball_front:
    def __init__(self):
        rospy.init_node('hsvFront_ball', anonymous=False)
        self.bridge = CvBridge()
        # FIX: sebelumnya subscribe ke '/barracuda_vision/camera/front_raw'
        # (dari front_pub.py). Sekarang front_pub.py udah gak dipakai lagi
        # (diganti usb_cam driver di usb_cam.launch, biar gak rebutan
        # device kamera). Topic ini juga udah lewat front_flip_fix.py,
        # jadi orientasinya udah bener (gak kebalik atas-bawah lagi).
        self.image_get = rospy.Subscriber('/front/usb_cam/image_raw', Image, self.hsv_callback)
        # FIX: dikasih suffix "_hsv" biar ini jadi topic MENTAH (bukan
        # final). ballStatus sebelumnya malah SAMA PERSIS sama topic yg
        # dipakai process_front.py (YOLO) -- kalau dua-duanya jalan bareng,
        # itu tabrakan publisher. Sekarang node vision_fusion.py yang
        # gabungin hasil sini sama hasil YOLO, baru publish ke topic final.
        self.ballInfo_pub = rospy.Publisher('/barracuda_vision/camera/front/ballInfo_hsv', ballInfo, queue_size=10)
        self.ballTravel_pub = rospy.Publisher('/barracuda_vision/camera/front/ballTravel_hsv', ballTravel, queue_size=10)
        self.ballStatus_pub = rospy.Publisher("/barracuda_vision/camera/front/ballStatus_hsv", Bool, queue_size=1)
        self.hsvFront_pub = rospy.Publisher('/barracuda_vision/camera/front_hsv_mask', Image, queue_size=10)
        self.hsvMergeFront_pub = rospy.Publisher('/barracuda_vision/camera/front_hsv_merge', Image, queue_size=10)

    def hsv_callback(self, msg):
        global mode
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if cv_image is None:
            rospy.logwarn("Frame tidak diterima")
        height, width = cv_image.shape[:2]
        start_y = height // 2 
        end_y = height

        cv_image[365:end_y, 0:width] = 0
        if mode == 0:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Buat mask berdasarkan range HSV
            lower_hsv = np.array([H_min, S_min, V_min])
            upper_hsv = np.array([H_max, S_max, V_max])

            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.hsvFront_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
                if radius > rad_ball:
                    self.ballStatus_pub.publish(1)
                    ball_point.x = int(x)
                    ball_point.y = int(y)
                    ball_point.radius = int(radius)
                    ball_pos.distance = (int(getDistance(focalCam, ball_size, ball_point.radius * 2)) - adjDistance)
                    ball_pos.degree = getAngle(centerF_x, 360, ball_point.x, ball_point.y)
                    if ball_point.y > 343 and ball_pos.distance > 40:
                        ball_pos.distance = 40
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
            self.hsvMergeFront_pub.publish(self.bridge.cv2_to_imgmsg(masked_frame, "bgr8"))
        elif mode == 1:
            pass
        elif mode == 2:
            pass
        else:
            mode = 0

if __name__ == "__main__":
    try:
        frontRun = hsvball_front()
        rospy.spin()
    except rospy.ROSInterruptException:  # Corrected exception name
        pass
