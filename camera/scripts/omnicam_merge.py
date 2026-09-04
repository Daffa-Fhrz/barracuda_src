#!/usr/bin/env python3
# ==============================================
# Bagian Import Module
# ==============================================
import rospy
import cv2
from sensor_msgs.msg import Image
from camera.msg import ballPoint, ballStraightPoint
from cv_bridge import CvBridge
import numpy as np
import math

# ==============================================
# Bagian Inisialisasi
# ==============================================
focal = rospy.get_param('~focal_lenght', 100)
ballSize = rospy.get_param('ball_radius_cm', default=15)
# ==============================================
# Bagian Class
# ==============================================
class mergeCam_omni:
    # INITIALIZATION SECTION
    def __init__(self):
        # Inisialisasi variabel
        self.pos_x = None
        self.pos_y = None
        self.r_value = None
        self.angle = None
        self.distance = None
        self.centerF_x = 640 / 2
        self.centerF_Y = 480 / 2

        # --NODE INITIALIZATION--
        rospy.init_node('omnicam_view', anonymous=False)
        self.bridge = CvBridge()

        # --SUBSCRIBER TOPIC--(CAMERA, HSV, DUMMY,  )
        self.ballPos = rospy.Subscriber('/ballPos/omniBall', ballPoint, self.straightPos_callback)
        self.image_get = rospy.Subscriber('/camera/omni_raw', Image, self.image_callback)
        # --PUBLISHER TOPIC--
        self.straightPos = rospy.Publisher('/straightPos_Deg/omniCam', ballStraightPoint, queue_size=8)

    # ==========================================
    # Fungsi Callback
    # ==========================================
    def image_callback(self, msg):
        try:
            # Konversi pesan ROS ke OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Gambar lingkaran dan garis jika data posisi bola valid
            if self.pos_x is not None and self.pos_y is not None and self.r_value is not None:
                # Gambar lingkaran dan garis
                ballCircle = cv2.circle(frame, (int(self.pos_x), int(self.pos_y)), int(self.r_value), (0, 255, 0), 2)
                trajectBall = cv2.line(frame, (int(self.centerF_x), int(self.centerF_Y)), (int(self.pos_x), int(self.pos_y)), (255, 0, 0), 2)
            else :
                self.pos_x = None
                self.pos_y = None
                self.r_value = None
            if self.angle  is not None and self.distance is not None:
                text_angle = f"Angle: {self.angle:.2f} deg"
                text_distance = f"Distance: {self.distance:.2f} cm"
                cv2.putText(frame, text_angle, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, text_distance, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            elif self.angle is None and self.distance is None:
                text_angle = f"Angle: no balls"
                text_distance = f"Distance: no balls"
                cv2.putText(frame, text_angle, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, text_distance, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.line(frame, (int(self.centerF_x), 0), (int(self.centerF_x), 480), (255, 255, 255), 1)
            cv2.line(frame, (0, int(self.centerF_Y)), (640, int(self.centerF_Y)), (255, 255, 255), 1)
            cv2.imshow("Omnicam View", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error dalam image_callback: {e}")

    def straightPos_callback(self, msg):
        try:
            # Simpan nilai posisi bola
            if msg.radius > 10:  # Hanya simpan jika radius bola valid
                self.pos_x = msg.x
                self.pos_y = msg.y
                self.r_value = msg.radius
            else:
                # Reset nilai posisi bola jika bola tidak terdeteksi
                self.pos_x = None
                self.pos_y = None
                self.r_value = None

            # Hitung sudut dan jarak
            if self.pos_x is not None and self.pos_y is not None and self.r_value is not None:
                self.angle = self.getAngle(self.centerF_x, self.centerF_Y, self.pos_x, self.pos_y)
                self.distance = self.getDistance(focal, ballSize, self.r_value * 2)

                # Buat pesan dan publikasikan
                goPos = ballStraightPoint()
                goPos.distance = self.distance
                goPos.degree = self.angle
                self.straightPos.publish(goPos)
            else:
                self.angle = None
                self.distance = None

        except Exception as e:
            rospy.logerr(f"Error dalam straightPos_callback: {e}")

    # ==========================================
    # Fungsi Utama
    # ==========================================
    def getAngle(self, center_x, center_y, obj_x, obj_y):
        dx = obj_x - center_x
        dy = center_y - obj_y
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg += 360
        return angle_deg

    def getDistance(self, focal_length, real_size, pixel_size):
        return (focal_length * real_size) / pixel_size

    # RUNNING MAIN
    def mainRun(self):
        rospy.spin()
        cv2.destroyAllWindows()


# ==============================================
# Program Berjalan (if __name__ == '__main__')
# ==============================================
if __name__ == '__main__':
    try:
        omni = mergeCam_omni()  # Buat instance dari kelas
        omni.mainRun()  # Jalankan mainRun
    except rospy.ROSInterruptException:
        pass