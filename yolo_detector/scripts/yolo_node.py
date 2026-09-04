#!/usr/bin/env python3

import rospy
import cv2
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class YOLODetector:
    def __init__(self, model_path):
        self.bridge = CvBridge()

        # Load TensorRT model
        self.model = YOLO(model_path)

        rospy.loginfo(f"Loaded TensorRT model: {model_path}")

        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_callback,
            queue_size=1
        )

        self.image_pub = rospy.Publisher(
            "/yolo_detections/image_out",
            Image,
            queue_size=1
        )

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        results = self.model(frame)[0]

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])

            if conf < 0.25:
                continue

            label = f"{cls}:{conf:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, label, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("yolo_detector_node")

    model_path = rospy.get_param(
        "~model_path",
        "/home/barracuda/Documents/best.engine"
    )

    YOLODetector(model_path)
    rospy.spin()