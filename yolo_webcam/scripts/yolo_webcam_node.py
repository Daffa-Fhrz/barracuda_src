#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

class YOLO_Webcam_ROS:
    def __init__(self):
        rospy.init_node("yolo_webcam_node")

        self.bridge = CvBridge()

        # Load YOLOv8
        self.model = YOLO("yolov8n.pt")  # pastikan ada di folder yang sama atau pakai path lengkap

        # Buka webcam
        self.cap = cv2.VideoCapture(0)

        # ROS publisher
        self.pub = rospy.Publisher("/yolo_webcam/image", Image, queue_size=1)

        rospy.loginfo("YOLO Webcam ROS started.")
        self.loop()

    def loop(self):
        rate = rospy.Rate(30)  # 30 FPS
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Webcam error!")
                continue

            # YOLO inference
            results = self.model(frame)[0]
            annotated = results.plot()

            # Publish image
            msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            self.pub.publish(msg)

            rate.sleep()

if __name__ == "__main__":
    YOLO_Webcam_ROS()
