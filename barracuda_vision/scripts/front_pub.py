#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

LEBAR_FRAME = 640
TINGGI_FRAME = 480
FPS_FRAME = 30

camera_index = rospy.get_param('~camera_index', 0)
flip_cam = rospy.get_param('~flip_cam', -1)
flipC_status = rospy.get_param('~flip_status', True)
image_pub = rospy.Publisher('/barracuda_vision/camera/front_raw', Image, queue_size=10)
if __name__ == "__main__":
    rospy.init_node('front_publish', anonymous=False)
    bridge = CvBridge()
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        rospy.logerr(f"Tidak dapat membuka kamera dengan index: {camera_index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LEBAR_FRAME)   
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TINGGI_FRAME) 
    cap.set(cv2.CAP_PROP_FPS, FPS_FRAME)         

    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(30) 
    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logerr("Tidak dapat membaca frame.")
                break
            if flipC_status is True:
                frame = cv2.flip(frame, flip_cam)
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            rate.sleep()
        cap.release()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass