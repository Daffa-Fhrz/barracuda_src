#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import time

class YOLOTRT:
    def __init__(self):
        rospy.init_node("yolo_trt", anonymous=True)
        model_path = "/home/barracuda/catkin_ws/src/ultralytics_ros/models/best.engine"

        self.bridge = CvBridge()

        self.model = YOLO(model_path)
        self.sub = rospy.Subscriber("/front/usb_cam/image_raw", Image, self.image_callback)

        self.pub = rospy.Publisher('/yolo/detections', Image, queue_size=10)

        self.last_time = time.time()
        self.fps = 0.0

    def image_callback(self, msg):
        start = time.time() # start, untuk mengukur fps
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # result = self.model.track(cv_image, show=True)a
        result = self.model.predict(cv_image)

        annotated_frame = result[0].plot()

        # rospy.loginfo_throttle(2, f"annotated type: {type(annotated_frame)} shape: {annotated_frame.shape}")
        # perhitungan fps
        end = time.time()
        time_sec = (end - start)
        self.fps = fps = 1.0 / time_sec

            # 🔹 TULIS FPS KE GAMBAR
        cv2.putText(
            annotated_frame,
            f"fps: {fps:.2f}",
            (10, 30),                     # posisi (x, y)
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,                           # ukuran font
            (0, 0, 255),                   # warna (BGR)
            2                              # ketebalan
        )
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
 
        self.pub.publish(img_msg)

if __name__ == '__main__':
    try:
        node = YOLOTRT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
