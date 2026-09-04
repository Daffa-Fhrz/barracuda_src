#!/usr/bin/env python3
import rospy
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolo_barra.msg import YoloResult, ballInfo, ballTravel
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from yolo_barra.shapegui import line, text, crossPlus

class algo:
    def __init__(self):
        rospy.init_node("YoloProcessBarra")
        self.yolo = rospy.Subscriber("/yolo_result", YoloResult, self.dataCallback)
        self.image_get = rospy.Subscriber('/yolo_image', Image, self.mergeCallback)
        self.infopub = rospy.Publisher('/Barracuda_Yolo/ballInfo', ballInfo, queue_size=10)
        self.travelpub = rospy.Publisher('/Barracuda_Yolo/ballTravel', ballTravel, queue_size=10)
        self.mergepub = rospy.Publisher('/Barracuda_Yolo/MergeResult', Image, queue_size=10)
        self.result = YoloResult()
        self.infobola = ballInfo()
        self.travelbola = ballTravel()
        self.bridge = CvBridge()

        self.CF_x = 320
        self.CF_y = 240
        self.ballsize = 1.0
        self.focal_length = 5000
        self.adjdistance = 10

    def dataCallback(self, msg):
        if not msg.detections.detections:
            self.infobola.score = 0  # Reset jika tidak ada deteksi
            return
        for detection in msg.detections.detections:
            self.infobola.x = int(detection.bbox.center.x)
            self.infobola.y = int(detection.bbox.center.y)
            self.infobola.radius = int(int(detection.bbox.size_y) / 2)
            if detection.results:
                self.infobola.score = detection.results[0].score
            else:
                self.infobola.score = 0
            self.infopub.publish(self.infobola)
    
    def getTravel(self): 
        dx = self.infobola.x-self.CF_x
        dy = self.CF_y-self.infobola.y
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg += 360
        self.travelbola.degree = int(angle_deg)
        self.travelbola.distance = (self.focal_length * self.ballsize) / (self.infobola.radius * 2)
        self.travelpub.publish(self.travelbola)

    def mergeCallback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if cv_image is None:
            rospy.logwarn("Frame tidak diterima")
            return
        frame_ui = cv_image
        crossPlus(frame_ui)
        if self.infobola.score != 0:
            self.getTravel()
            line(frame_ui, 640 // 2, 480 // 2, self.infobola.x, self.infobola.y, "BIRU")
            text(frame_ui, f"x:{self.infobola.x}, y:{self.infobola.y}, rad:{self.infobola.radius}", 20, 40, "SIMPLEX", 0.5, "HIJAU", 1)
            text(frame_ui, f"distance:{self.travelbola.distance}, degree:{self.travelbola.degree}", 20, 70, "SIMPLEX", 0.5, "HIJAU", 1)
            text(frame_ui, f"BALL", 600, 40, "SIMPLEX", 0.5, "HIJAU", 2)
        else:
            text(frame_ui, f"x:NONE, y:NONE, rad:NONE", 20, 40, "SIMPLEX", 0.5, "MERAH", 1)
            text(frame_ui, f"distance:NONE, degree:NONE", 20, 70, "SIMPLEX", 0.5, "MERAH", 1)
            text(frame_ui, f"BALL", 600, 40, "SIMPLEX", 0.5, "MERAH", 2)
        self.mergepub.publish(self.bridge.cv2_to_imgmsg(frame_ui, "bgr8"))

if __name__ == "__main__":
    try:
        algo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
