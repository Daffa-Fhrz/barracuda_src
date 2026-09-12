#!/usr/bin/env python3
"""
front_flip_fix.py

Benerin orientasi frame mentah dari kamera front yang fisiknya kepasang
kebalik (atas jadi bawah). Subscribe dari topic "image_raw_orig" (hasil
remap dari usb_cam driver di usb_cam.launch), flip pakai OpenCV, terus
publish balik ke topic "image_raw" -- nama topic ASLI yang udah ditunggu
sama TrackerNode (yolo dual-cam), hsvFront.py, dan front_merge.py. Jadi
gak ada satupun consumer lain yang perlu diubah.

Kenapa gak pakai param "flip" bawaan driver usb_cam_node? Karena udah
dicoba (value=2) dan hasilnya masih kebalik -- kemungkinan enum/konvensi
param itu di driver ini gak sesuai ekspektasi (beda fork/versi usb_cam
punya konvensi beda-beda, gak selalu didokumentasiin). Dengan flip
eksplisit di sini, kita full kontrol & gampang di-tuning tanpa nebak-nebak.
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FrontFlipFix:
    def __init__(self):
        rospy.init_node('front_flip_fix', anonymous=False)
        self.bridge = CvBridge()

        # flip_code ikutin konvensi cv2.flip():
        #   0  -> flip vertikal   (atas jadi bawah)   <- default, buat kasus ini
        #   1  -> flip horizontal (kiri jadi kanan)
        #  -1  -> flip vertikal + horizontal (rotasi 180 derajat penuh)
        # Kalau abis dites flip_code=0 ternyata masih salah (atau malah
        # jadi kebalik kiri-kanan), tinggal ganti param "flip_code" di
        # usb_cam.launch (node front_flip_fix) ke 1 atau -1 -- TANPA perlu
        # ubah/rebuild kode ini.
        self.flip_code = rospy.get_param('~flip_code', 0)
        rospy.loginfo(f"[front_flip_fix] pakai flip_code={self.flip_code}")

        self.pub = rospy.Publisher('image_raw', Image, queue_size=10)
        rospy.Subscriber('image_raw_orig', Image, self.callback, queue_size=1)

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"[front_flip_fix] gagal convert image: {e}")
            return

        fixed = cv2.flip(cv_image, self.flip_code)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(fixed, encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"[front_flip_fix] gagal convert balik ke Image msg: {e}")
            return

        # Header (termasuk timestamp) dipertahankan dari frame asli, biar
        # sinkronisasi waktu di consumer (TrackerNode dll) tetep akurat.
        out_msg.header = msg.header
        self.pub.publish(out_msg)


if __name__ == '__main__':
    try:
        FrontFlipFix()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
