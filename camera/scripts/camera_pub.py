#!/usr/bin/env python3
# ==============================================
# Bagian Import Module
# ==============================================
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# ==============================================
# Bagian Inisialisasi
# ==============================================
LEBAR_FRAME = 640
TINGGI_FRAME = 480
FPS_FRAME = 30

# ==============================================
# Bagian Class
# ==============================================

    # ==========================================
    # Fungsi Callback
    # ==========================================

    # ==========================================
    # Fungsi Utama
    # ==========================================

# ==============================================
# Fungsi Callback
# ==============================================


# ==============================================
# Bagian Fungsi Utama
# ==============================================
def main():
    rospy.init_node('cam_publish', anonymous=False)
    camera_index = rospy.get_param('~camera_index', 0)
    rospy.loginfo(f"Menggunakan kamera dengan index: {camera_index}")
    if camera_index == 0:
        image_pub = rospy.Publisher('/camera/omni_raw', Image, queue_size=10)
    elif camera_index == 2:
        image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

    bridge = CvBridge()

    # Buka kamera berdasarkan index
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        rospy.logerr(f"Tidak dapat membuka kamera dengan index: {camera_index}")
        return
    
    #PENGATURAN FRAME
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LEBAR_FRAME)   
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TINGGI_FRAME) 
    cap.set(cv2.CAP_PROP_FPS, FPS_FRAME)         

    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)

    rate = rospy.Rate(30) 
    while not rospy.is_shutdown():
        # Baca frame dari kamera
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Tidak dapat membaca frame.")
            break

        # Publish frame ke topik ROS
        image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        rate.sleep()

    # Lepaskan kamera dan tutup jendela
    cap.release()
    cv2.destroyAllWindows()


# ==============================================
# Program Berjalan (if __name__ == '__main__')
# ==============================================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass