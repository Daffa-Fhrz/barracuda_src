#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from barracuda_vision.msg import ballInfo, ballTravel
from cv_bridge import CvBridge
import math
import shapegui as sp

font = cv2.FONT_HERSHEY_SIMPLEX
fontSize = 1.5
thickFont = 2

# Rename the global instances to avoid conflict with the message classes
ball_info_instance = ballInfo()
ball_travel_instance = ballTravel()
ball_status_instance = False
#--FUNCTION
def crossPlus(frameSet, W_cam = 640, H_cam = 480, colour_name = "HIJAU", thickness = 1):
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }   
    colour = WARNA.get(colour_name, (0, 255, 0))
    cv2.line(frameSet, (W_cam//2, 0), (W_cam//2, 480), colour, thickness) #VERTICAL
    cv2.line(frameSet, (0, H_cam//2), (640, H_cam//2), colour, thickness) #HORIZONTAL
    return frameSet

def circle(frameSet, center_x, center_y, radius, coulour_name, thickness = 1):
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }
    colour = WARNA.get(coulour_name, (0, 255, 0)) 
    cv2.circle(frameSet, (center_x, center_y),radius, colour, thickness)
    return frameSet

def line(frameSet, x_awal, y_awal, x_akhir, y_akhir, colour_name, thickness=1):
    """
    MENAMBAHKAN GARIS PADA DISPLAY
    :PILIHAN WARNA(HIJAU, BIRU, MERAH, HITAM, PUTIH)
    :WAJIB CAPS
    """
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }
    
    colour = WARNA.get(colour_name, (0, 255, 0))
    cv2.line(frameSet, (x_awal, y_awal), (x_akhir, y_akhir), colour, thickness)
    return frameSet

def text(frameSet, string, posX, posY, fontType, fontSize, colour_name= "HIJAU", fontThick= 1):
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    FONT = {
        "SIMPLEX": cv2.FONT_HERSHEY_SIMPLEX,
        "PLAIN": cv2.FONT_HERSHEY_PLAIN,
        "DUPLEX": cv2.FONT_HERSHEY_DUPLEX,
        "COMPLEX": cv2.FONT_HERSHEY_COMPLEX,
        "TRIPLEX": cv2.FONT_HERSHEY_TRIPLEX,
        "COMPLEX_SMALL": cv2.FONT_HERSHEY_COMPLEX_SMALL,
        "SCRIPT_SIMPLEX": cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,
        "SCRIPT_COMPLEX": cv2.FONT_HERSHEY_SCRIPT_COMPLEX
    }
    font = FONT.get(fontType, cv2.FONT_HERSHEY_SIMPLEX)
    
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }   
    colour = WARNA.get(colour_name, (0, 255, 0))
    cv2.putText(frameSet, string,(posX,posY), font, fontSize, colour, fontThick)
    return frameSet
    
#--CLASS--
class mergeOmni:
    def __init__(self):
        rospy.init_node('omnicam_view', anonymous=False)
        self.bridge = CvBridge()
        # FIX: sebelumnya subscribe ke '/barracuda_vision/camera/omni_raw'
        # (dari omni_pub.py, udah gak dipakai lagi). Sekarang langsung ke
        # topic usb_cam.
        self.image_get = rospy.Subscriber('/omni/usb_cam/image_raw', Image, self.mergeView)
        # FIX: disesuaikan ke topic HSV mentah yang baru (_hsv), biar
        # debug view ini nunjukin hasil HSV yang sebenernya (bukan hasil
        # fusion final yang sekarang ada di /barracuda_vision/camera/omni/ballInfo dst)
        self.ballInfo_get = rospy.Subscriber('/barracuda_vision/camera/omni/ballInfo_hsv', ballInfo, self.ballInfo_callback)
        self.ballTravel_get = rospy.Subscriber('/barracuda_vision/camera/omni/ballTravel_hsv', ballTravel, self.ballTravel_callback)
        self.ballStatus_get = rospy.Subscriber("/barracuda_vision/camera/omni/ballStatus_hsv", Bool, self.ballStatus_callback)
        self.image_pub = rospy.Publisher('/barracuda_vision/camera/omni/merge', Image, queue_size=10)

    def ballInfo_callback(self, data):
        global ball_info_instance, ball_status_instance
        if ball_status_instance:
            ball_info_instance = data
        else:
            ball_info_instance.x = 0
            ball_info_instance.y = 0
            ball_info_instance.radius = 0

    def ballTravel_callback(self, data):
        global ball_travel_instance, ball_status_instance
        if ball_status_instance:
            ball_travel_instance = data
        else:
            ball_travel_instance.distance = 0
            ball_travel_instance.degree = 0

    def ballStatus_callback(self, data):
        global ball_status_instance
        ball_status_instance = data.data

    def mergeView(self, msg):
        global font, fontSize, thickFont, ball_info_instance, ball_status_instance, ball_travel_instance
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if cv_image is None:
                rospy.logwarn("Frame tidak diterima")
                return
            frame_ui = cv_image
            crossPlus(frame_ui)
            if ball_status_instance:
                circle(frame_ui, ball_info_instance.x, ball_info_instance.y, ball_info_instance.radius, "HIJAU", 1)
                line(frame_ui, 640 // 2, 480 // 2, ball_info_instance.x, ball_info_instance.y, "BIRU", 2)
                text(frame_ui, f"x:{ball_info_instance.x}, y:{ball_info_instance.y}, rad:{ball_info_instance.radius}", 20, 40, "SIMPLEX", 0.5, "HIJAU", 1)
                text(frame_ui, f"distance:{ball_travel_instance.distance}, degree:{ball_travel_instance.degree}", 20, 70, "SIMPLEX", 0.5, "HIJAU", 1)
                text(frame_ui, f"BALL", 600, 40, "SIMPLEX", 0.5, "HIJAU", 2)
            else:
                text(frame_ui, f"BALL", 600, 40, "SIMPLEX", 0.5, "MERAH", 2)
                text(frame_ui, f"x:NONE, y:NONE, rad:NONE", 20, 40, "SIMPLEX", 0.5, "MERAH", 1)
                text(frame_ui, f"distance:NONE, degree:NONE", 20, 70, "SIMPLEX", 0.5, "MERAH", 1)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame_ui, "bgr8"))
        except Exception as e:
            rospy.logerr(f"Error in mergeView: {e}")

if __name__ == "__main__":
    try:
        merge = mergeOmni()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass