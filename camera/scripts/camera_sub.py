#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_callback(msg):
    try:
        # Inisialisasi cv_bridge
        bridge = CvBridge()

        # Konversi gambar ROS ke OpenCV
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Tampilkan gambar di window
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)  # Tunggu 1 ms untuk memperbarui window

    except Exception as e:
        rospy.logerr(f"Error dalam callback: {e}")

def main():
    # Inisialisasi node ROS
    rospy.init_node('camera_subscriber', anonymous=False)

    # Subscribe ke topik /camera/image_raw
    rospy.Subscriber('/camera/image_raw', Image, image_callback)

    # Jaga agar node tetap berjalan
    rospy.spin()

    # Tutup jendela OpenCV saat node dihentikan
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass