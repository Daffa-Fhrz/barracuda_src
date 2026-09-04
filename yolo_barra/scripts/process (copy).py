#!/usr/bin/env python3

import rospy
import math
import threading
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolo_barra.msg import YoloResult
from yolo_barra.shapegui import line, text, crossPlus


# ============================================================
# KAMERA : ELP-USBGS1200P01 (Aptina AR0234, 1/2.6", GS)
#
# Pixel size  : 3.0 µm
# Resolusi    : 1920 x 1200
# Sensor W    : 1920 * 3.0 µm = 5.76 mm
# Sensor H    : 1200 * 3.0 µm = 3.60 mm
#
# Focal length dalam PIXEL:
#   f_px = (f_mm / sensor_width_mm) * image_width_px
#
# Rentang lensa (2.8–12 mm zoom):
#   f_min (2.8 mm) = (2.8 / 5.76) * 1920 ≈  933 px
#   f_max (12.0 mm)= (12.0 / 5.76) * 1920 ≈ 4000 px
#
# Sesuaikan FOCAL_LENGTH_MM dengan posisi zoom aktual.
# ============================================================

PIXEL_SIZE_MM   = 0.003          # 3.0 µm
IMAGE_W         = 1920
IMAGE_H         = 1200
SENSOR_W_MM     = IMAGE_W * PIXEL_SIZE_MM   # 5.76 mm
FOCAL_LENGTH_MM = 2.8            # ← ganti sesuai zoom (2.8–12 mm)

def mm_to_px(f_mm):
    return (f_mm / SENSOR_W_MM) * IMAGE_W


class Algo:

    def __init__(self):

        rospy.init_node("YoloProcessBarra")

        # --------------------------------------------------
        # FOCAL LENGTH (dalam pixel)
        # Bisa di-override lewat ROS param:
        #   rosparam set /yolo_process_barra/focal_mm 6.0
        # --------------------------------------------------
        f_mm = rospy.get_param("~focal_mm", FOCAL_LENGTH_MM)
        self.focal_px = mm_to_px(f_mm)
        rospy.loginfo(f"[KAMERA] focal={f_mm} mm  →  {self.focal_px:.1f} px")

        # RESOLUSI & CENTER (baca dari param agar mudah ganti)
        self.img_w = rospy.get_param("~img_w", IMAGE_W)
        self.img_h = rospy.get_param("~img_h", IMAGE_H)
        self.CF_x  = 320
        self.CF_y  = 240

        # --------------------------------------------------
        # UKURAN OBJEK NYATA (cm) — sesuaikan dengan arena
        # --------------------------------------------------
        self.object_size = {
            "BALL"   : 21.5,   # bola RoboCup diameter 21.5 cm
            "ROBOT"  : 40.0,   # tinggi robot (bbox height)
            "GAWANG" : 50.0,   # lebar tiang gawang
            "DUMMY"  : 30.0,
        }

        # CLASS ID
        self.class_names = {
            0: "BALL",
            1: "ROBOT",
            2: "GAWANG",
            3: "DUMMY",
        }

        # WARNA UI
        self.object_color = {
            "BALL"   : "HIJAU",
            "ROBOT"  : "BIRU",
            "GAWANG" : "KUNING",
            "DUMMY"  : "UNGU",
        }

        # DATA OBJEK — list per kelas agar bisa >1 robot
        self._lock   = threading.Lock()
        self.objects = {}           # { class_name: [data, ...] }

        self.bridge = CvBridge()

        # SUBSCRIBER
        self.sub_yolo  = rospy.Subscriber("/yolo_result",  YoloResult, self.dataCallback)
        self.sub_image = rospy.Subscriber("/yolo_image",   Image,      self.mergeCallback)

        # PUBLISHER
        self.pub_merge = rospy.Publisher(
            "/Barracuda_Yolo/MergeResult", Image, queue_size=10
        )

    # =========================================================
    # CALLBACK YOLO
    # =========================================================

    def dataCallback(self, msg):

        new_objects = {}

        if not msg.detections.detections:
            with self._lock:
                self.objects = {}
            return

        for detection in msg.detections.detections:

            if not detection.results:
                continue

            class_id = detection.results[0].id
            if class_id not in self.class_names:
                continue

            class_name = self.class_names[class_id]
            score      = detection.results[0].score

            # Pusat bbox
            bx = int(detection.bbox.center.x)
            by = int(detection.bbox.center.y)

            # Ukuran bbox
            bw = int(detection.bbox.size_x)
            bh = int(detection.bbox.size_y)

            # Gunakan sisi terbesar bbox sebagai referensi jarak
            # agar lebih stabil (tidak terpengaruh oklusi)
            bbox_ref = max(bw, bh)

            # ── JARAK (cm) ──────────────────────────────────────
            # d = (f_px * ukuran_nyata) / ukuran_pixel
            if bbox_ref > 0:
                distance = (
                    self.focal_px * self.object_size[class_name]
                ) / bbox_ref
            else:
                distance = 0.0

            # ── SUDUT TERHADAP PUSAT FRAME ───────────────────────
            # 0° = kanan, 90° = atas, 180° = kiri, 270° = bawah
            dx = bx - self.CF_x
            dy = self.CF_y - by          # sumbu-Y layar terbalik
            angle_deg = math.degrees(math.atan2(dy, dx)) % 360

            # Kumpulkan per kelas (bisa >1 instansi)
            entry = {
                "x"        : bx,
                "y"        : by,
                "w"        : bw,
                "h"        : bh,
                "score"    : score,
                "distance" : distance,
                "degree"   : angle_deg,
            }

            new_objects.setdefault(class_name, []).append(entry)

        # Tulis atomik agar mergeCallback tidak baca data setengah
        with self._lock:
            self.objects = new_objects

    # =========================================================
    # CALLBACK IMAGE
    # =========================================================

    def mergeCallback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if cv_image is None:
            rospy.logwarn("Frame tidak diterima")
            return

        frame_ui = cv_image.copy()

        crossPlus(frame_ui)

        # Snapshot supaya tidak terkunci lama
        with self._lock:
            snapshot = {k: list(v) for k, v in self.objects.items()}

        for class_name, detections in snapshot.items():
            color = self.object_color[class_name]

            for idx, data in enumerate(detections):

                bx = data["x"]
                by = data["y"]
                bw = data["w"]
                bh = data["h"]

                # Gambar bounding box
                self._draw_bbox(frame_ui, bx, by, bw, bh, color)

                # Garis dari pusat frame ke objek
                line(frame_ui, self.CF_x, self.CF_y, bx, by, color)

                # Label: nama kelas + indeks jika >1
                label = class_name if len(detections) == 1 else f"{class_name}#{idx+1}"
                offset_y = by - bh // 2 - 10   # di atas bbox

                text(frame_ui, label,
                     bx, offset_y, "SIMPLEX", 0.55, color, 2)

                # Info detail di bawah bbox
                base_y = by + bh // 2 + 15
                info_lines = [
                    f"x:{bx} y:{by}",
                    f"dist:{data['distance']:.1f} cm",
                    f"deg:{data['degree']:.1f}",
                    f"conf:{data['score']:.2f}",
                ]
                for i, info in enumerate(info_lines):
                    text(frame_ui, info,
                         bx, base_y + i * 18, "SIMPLEX", 0.45, color, 1)

        # Tampilkan info kamera di pojok kiri atas
        cam_info = (
            f"f={self.focal_px:.0f}px | "
            f"res={self.img_w}x{self.img_h} | "
            f"obj:{sum(len(v) for v in snapshot.values())}"
        )
        cv2.putText(
            frame_ui, cam_info, (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1
        )

        self.pub_merge.publish(
            self.bridge.cv2_to_imgmsg(frame_ui, "bgr8")
        )

    # =========================================================
    # HELPER
    # =========================================================

    def _draw_bbox(self, frame, cx, cy, bw, bh, color_name):
        """Gambar bounding-box dari pusat + ukuran."""
        color_map = {
            "HIJAU"  : (0,   255,   0),
            "BIRU"   : (255,   0,   0),
            "KUNING" : (0,   255, 255),
            "UNGU"   : (255,   0, 255),
        }
        bgr = color_map.get(color_name, (255, 255, 255))

        x1 = cx - bw // 2
        y1 = cy - bh // 2
        x2 = cx + bw // 2
        y2 = cy + bh // 2

        cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)

        # Sudut tebal untuk kejelasan
        corner = 10
        thick  = 3
        for px, py, sx, sy in [
            (x1, y1,  1,  1),
            (x2, y1, -1,  1),
            (x1, y2,  1, -1),
            (x2, y2, -1, -1),
        ]:
            cv2.line(frame, (px, py), (px + sx * corner, py), bgr, thick)
            cv2.line(frame, (px, py), (px, py + sy * corner), bgr, thick)


if __name__ == "__main__":
    try:
        Algo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass