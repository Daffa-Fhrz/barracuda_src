#!/usr/bin/env python3
# ==============================================
# Bagian Import Module
# ==============================================
import rospy
import cv2
from sensor_msgs.msg import Image
from camera.msg import ballPoint
from cv_bridge import CvBridge
import numpy as np
import math

# ==============================================
# Bagian Inisialisasi
# ==============================================


# ==============================================
# Bagian Class
# ==============================================
class hsv_pub:
    def __init__(self):
        # --NODE INITIALIZATION--
        rospy.init_node('hsv_pub', anonymous=False)
        self.indexCam = rospy.get_param('~index', 0)
        self.bridge = CvBridge()

        # --SUBSCRIBER TOPIC--
        if self.indexCam == 0:
            self.image_get = rospy.Subscriber('/camera/omni_raw', Image, self.image_callback)
        elif self.indexCam == 1:
            self.image_get = rospy.Subscriber('/camera/fish_raw', Image, self.image_callback)

        # --PUBLISHER TOPIC--
        if self.indexCam == 0:
            self.ballPos = rospy.Publisher('/ballPos/omniBall', ballPoint, queue_size=3)
        elif self.indexCam == 1:
            self.ballPos = rospy.Publisher('/ballPos/fishBall', ballPoint, queue_size=3)

    # ==========================================
    # Fungsi Callback
    # ==========================================
    def image_callback(self, msg):
        try:
            # Konversi pesan ROS ke OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Pastikan frame valid
            if cv_image is None:
                rospy.logwarn("Frame yang diterima tidak valid.")
                return

            # Konversi ke HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Buat mask berdasarkan range HSV
            lower_hsv = np.array([0, 110, 100])
            upper_hsv = np.array([15, 255, 255])
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

            # Operasi morfologi untuk membersihkan noise
            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=1)

            # Cari kontur pada mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Jika kontur ditemukan
            if contours:
                # Ambil kontur terbesar (asumsi itu adalah bola)
                largest_contour = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

                # Jika radius cukup besar, anggap itu bola
                if radius > 8:
                    center = ballPoint()
                    center.x = x
                    center.y = y
                    center.radius = radius
                    self.ballPos.publish(center)
            else:
                center = ballPoint()
                center.x = 480/2
                center.y = 640/2
                center.radius = 0
                self.ballPos.publish(center)
            cv2.imshow('masking', mask)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error dalam image_callback: {e}")

    # ==========================================
    # Fungsi Utama
    # ==========================================


# ==============================================
# Program Berjalan (if __name__ == '__main__')
# ==============================================
if __name__ == '__main__':
    try:
        detector = hsv_pub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass