#!/usr/bin/env python3
"""
vision_fusion.py

Node fusion: gabungin hasil deteksi YOLO (dari process_front.py /
process_omni.py, lewat topic mentah "*_yolo") sama hasil HSV color-mask
(dari hsvFront.py / hsvOmni.py, lewat topic mentah "*_hsv"), lalu publish
hasil gabungannya ke topic ASLI yang didengerin node kinematic
(IK_kinematic.py):

    /Barracuda_Yolo/front/ballInfo        (yolo_barra/ballInfo)
    /Barracuda_Yolo/front/ballTravel      (yolo_barra/ballTravel)
    /barracuda_vision/camera/front/ballStatus   (std_msgs/Bool)
    /Barracuda_Yolo/omni/ballInfo
    /Barracuda_Yolo/omni/ballTravel
    /barracuda_vision/camera/omni/ballStatus

STRATEGI FUSION (prioritas sederhana):
  1. Kalau YOLO detect bola          -> pakai data YOLO.
     (YOLO lebih tahan false-positive / bisa bedain bola vs distractor
     lain karena belajar dari dataset, bukan cuma warna.)
  2. Kalau YOLO GA detect, tapi HSV detect -> pakai data HSV sbg fallback.
     (Color-mask cenderung lebih tahan motion blur drpd deteksi berbasis
     bentuk/fitur kayak YOLO -- warnanya masih "nyebar" di situ walau
     bentuk bolanya udah kabur kena blur pas robot gerak cepat. Ini yang
     nutupin kelemahan YOLO yang jadi masalah utama kalian.)
  3. Kalau dua-duanya GA detect       -> beneran dianggap ga ada bola.

PENTING -- prasyarat sebelum jalanin node ini:
  - hsvFront.py / hsvOmni.py sudah dipatch supaya publish ke topic
    "..._hsv" (bukan topic asli lagi).
  - process_front.py / process_omni.py sudah dipatch supaya publish ke
    topic "..._yolo" (bukan topic asli lagi).
  - process_front.py & process_omni.py sudah dikasih nama node yang unik
    (sebelumnya dua-duanya "YoloProcessBarra", sekarang
    "YoloProcessBarra_front" / "YoloProcessBarra_omni").
  Kalau file-file itu belum dipatch, node fusion ini gak akan nerima data
  apa-apa (subscribe ke topic yang gak ada yang publish).

CATATAN TUNING:
  Kalau setelah dites ternyata HSV JUSTRU sering kasih posisi ngaco pas
  dipakai sbg fallback (misal salah nangkep warna lain), gampang: kondisi
  di _publish_fused() tinggal ditambah syarat, misal cuma percaya HSV
  kalau radius-nya di atas ambang tertentu, dsb.
"""

import rospy
from std_msgs.msg import Bool
from yolo_barra.msg import ballInfo as YoloBallInfo, ballTravel as YoloBallTravel
from barracuda_vision.msg import ballInfo as HsvBallInfo, ballTravel as HsvBallTravel


class CameraFusion:
    """Nanganin fusion buat SATU kamera. Dipakai 2x (front & omni) di
    bagian __main__ di bawah."""

    def __init__(self, cam_name,
                 yolo_ballinfo_topic, yolo_balltravel_topic, yolo_ballstatus_topic,
                 hsv_ballinfo_topic, hsv_balltravel_topic, hsv_ballstatus_topic,
                 out_ballinfo_topic, out_balltravel_topic, out_ballstatus_topic):
        self.cam_name = cam_name

        # State terakhir dari masing-masing sumber (di-update tiap ada
        # pesan baru dari subscriber-nya masing-masing)
        self.yolo_status = False
        self.yolo_info = YoloBallInfo()
        self.yolo_travel = YoloBallTravel()

        self.hsv_status = False
        self.hsv_info = HsvBallInfo()
        self.hsv_travel = HsvBallTravel()

        # --- Subscriber ke data MENTAH dari masing-masing pipeline ---
        rospy.Subscriber(yolo_ballstatus_topic, Bool, self._yolo_status_cb)
        rospy.Subscriber(yolo_ballinfo_topic, YoloBallInfo, self._yolo_info_cb)
        rospy.Subscriber(yolo_balltravel_topic, YoloBallTravel, self._yolo_travel_cb)

        rospy.Subscriber(hsv_ballstatus_topic, Bool, self._hsv_status_cb)
        rospy.Subscriber(hsv_ballinfo_topic, HsvBallInfo, self._hsv_info_cb)
        rospy.Subscriber(hsv_balltravel_topic, HsvBallTravel, self._hsv_travel_cb)

        # --- Publisher ke topic ASLI yang didengerin kinematic ---
        self.out_info_pub = rospy.Publisher(out_ballinfo_topic, YoloBallInfo, queue_size=10)
        self.out_travel_pub = rospy.Publisher(out_balltravel_topic, YoloBallTravel, queue_size=10)
        self.out_status_pub = rospy.Publisher(out_ballstatus_topic, Bool, queue_size=1)

        rospy.loginfo(f"[vision_fusion] siap fusion kamera '{cam_name}'")

    # ---- callback YOLO ----
    def _yolo_status_cb(self, msg):
        self.yolo_status = msg.data
        self._publish_fused()

    def _yolo_info_cb(self, msg):
        self.yolo_info = msg

    def _yolo_travel_cb(self, msg):
        self.yolo_travel = msg

    # ---- callback HSV ----
    def _hsv_status_cb(self, msg):
        self.hsv_status = msg.data
        self._publish_fused()

    def _hsv_info_cb(self, msg):
        self.hsv_info = msg

    def _hsv_travel_cb(self, msg):
        self.hsv_travel = msg

    # ---- fusion & publish ----
    def _publish_fused(self):
        """Dipanggil tiap salah satu status (yolo/hsv) update, biar hasil
        fusion selalu ngikutin status terbaru secepat mungkin (bukan
        nunggu timer/loop rate terpisah)."""
        if self.yolo_status:
            # YOLO detect -> prioritas utama
            fused_info = YoloBallInfo()
            fused_info.x = self.yolo_info.x
            fused_info.y = self.yolo_info.y
            fused_info.radius = self.yolo_info.radius

            fused_travel = YoloBallTravel()
            fused_travel.distance = self.yolo_travel.distance
            fused_travel.degree = self.yolo_travel.degree

            fused_status = True

        elif self.hsv_status:
            # YOLO ga detect (kemungkinan blur pas gerak cepat), TAPI HSV
            # masih nangkep -> pakai punya HSV sbg fallback.
            fused_info = YoloBallInfo()
            fused_info.x = self.hsv_info.x
            fused_info.y = self.hsv_info.y
            fused_info.radius = self.hsv_info.radius

            fused_travel = YoloBallTravel()
            fused_travel.distance = self.hsv_travel.distance
            fused_travel.degree = self.hsv_travel.degree

            fused_status = True

        else:
            # Dua-duanya ga detect -> beneran ga ada bola
            fused_info = YoloBallInfo()
            fused_travel = YoloBallTravel()
            fused_status = False

        self.out_status_pub.publish(Bool(data=fused_status))
        if fused_status:
            self.out_info_pub.publish(fused_info)
            self.out_travel_pub.publish(fused_travel)


if __name__ == '__main__':
    rospy.init_node('vision_fusion', anonymous=False)

    front_fusion = CameraFusion(
        cam_name='front',
        yolo_ballinfo_topic='/Barracuda_Yolo/front/ballInfo_yolo',
        yolo_balltravel_topic='/Barracuda_Yolo/front/ballTravel_yolo',
        yolo_ballstatus_topic='/barracuda_vision/camera/front/ballStatus_yolo',
        hsv_ballinfo_topic='/barracuda_vision/camera/front/ballInfo_hsv',
        hsv_balltravel_topic='/barracuda_vision/camera/front/ballTravel_hsv',
        hsv_ballstatus_topic='/barracuda_vision/camera/front/ballStatus_hsv',
        out_ballinfo_topic='/Barracuda_Yolo/front/ballInfo',
        out_balltravel_topic='/Barracuda_Yolo/front/ballTravel',
        out_ballstatus_topic='/barracuda_vision/camera/front/ballStatus',
    )

    omni_fusion = CameraFusion(
        cam_name='omni',
        yolo_ballinfo_topic='/Barracuda_Yolo/omni/ballInfo_yolo',
        yolo_balltravel_topic='/Barracuda_Yolo/omni/ballTravel_yolo',
        yolo_ballstatus_topic='/barracuda_vision/camera/omni/ballStatus_yolo',
        hsv_ballinfo_topic='/barracuda_vision/camera/omni/ballInfo_hsv',
        hsv_balltravel_topic='/barracuda_vision/camera/omni/ballTravel_hsv',
        hsv_ballstatus_topic='/barracuda_vision/camera/omni/ballStatus_hsv',
        out_ballinfo_topic='/Barracuda_Yolo/omni/ballInfo',
        out_balltravel_topic='/Barracuda_Yolo/omni/ballTravel',
        out_ballstatus_topic='/barracuda_vision/camera/omni/ballStatus',
    )

    rospy.spin()