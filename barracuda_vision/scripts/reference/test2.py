#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from threading import Thread

#--CLASS VIDEO (PENGATURAN VIDEO)--#
class VideoStream:
    def __init__(self, src=0, exposure_val=0, width=640, height=480):
        self.stream = cv2.VideoCapture(src)

        # Set properti kamera
        self.stream.set(cv2.CAP_PROP_FPS, 30)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.frame_width = int(self.stream.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rospy.loginfo(f"Frame Width: {self.frame_width}, Frame Height: {self.frame_height}")
        self.fps = self.stream.get(cv2.CAP_PROP_FPS)
        rospy.loginfo(f"FPS: {self.fps}")

        # Set eksposur dan autofokus (jika didukung)
        if self.stream.get(cv2.CAP_PROP_EXPOSURE) is not None:
            self.exposure_val = exposure_val
            self.stream.set(cv2.CAP_PROP_EXPOSURE, self.exposure_val)
        if self.stream.get(cv2.CAP_PROP_AUTOFOCUS) is not None:
            self.stream.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        # Inisialisasi frame
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        # Mulai thread untuk membaca frame
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Loop untuk membaca frame
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # Kembalikan frame terbaru
        return self.frame

    def stop(self):
        # Hentikan thread dan lepaskan kamera
        self.stopped = True
        self.stream.release()

#--FUNGSI UNTUK MENGATUR TRACKBAR--#
def nothing(x):
    pass

#--MAIN FUNCTION --#
if __name__ == '__main__':
    rospy.init_node('omni_cam')

    # Ambil parameter dari ROS parameter server
    cam_index = rospy.get_param("~cam_index", 0)
    exposure_val = rospy.get_param("~exposure_val", 0)
    vs = VideoStream(src=cam_index, exposure_val=exposure_val).start()

    # Buat window dan trackbar untuk mengatur nilai HSV
    cv2.namedWindow('Masking')
    cv2.createTrackbar('Hue Min', 'Masking', 15, 179, nothing)
    cv2.createTrackbar('Hue Max', 'Masking', 179, 179, nothing)
    cv2.createTrackbar('Sat Min', 'Masking', 150, 255, nothing)
    cv2.createTrackbar('Sat Max', 'Masking', 255, 255, nothing)
    cv2.createTrackbar('Val Min', 'Masking', 100, 255, nothing)
    cv2.createTrackbar('Val Max', 'Masking', 255, 255, nothing)

    try:
        while not rospy.is_shutdown():
            # Baca frame dari kamera
            frame = vs.read()
            if frame is None:
                rospy.logerr("Tidak dapat membaca frame.")
                break

            # Blur frame untuk mengurangi noise
            frame_ui = cv2.GaussianBlur(frame, (5, 5), 0)

            # Konversi frame ke HSV
            hsv = cv2.cvtColor(frame_ui, cv2.COLOR_BGR2HSV)

            # Ambil nilai HSV dari trackbar
            Hue_min = cv2.getTrackbarPos('Hue Min', 'Masking')
            Hue_max = cv2.getTrackbarPos('Hue Max', 'Masking')
            Sat_min = cv2.getTrackbarPos('Sat Min', 'Masking')
            Sat_max = cv2.getTrackbarPos('Sat Max', 'Masking')
            Val_min = cv2.getTrackbarPos('Val Min', 'Masking')
            Val_max = cv2.getTrackbarPos('Val Max', 'Masking')

            # Buat mask berdasarkan nilai HSV
            mask = cv2.inRange(hsv, (Hue_min, Sat_min, Val_min), (Hue_max, Sat_max, Val_max))

            # Aplikasikan mask ke frame asli
            masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

            # Tampilkan frame dan mask
            cv2.imshow("Original Frame", frame)
            cv2.imshow("HSV Frame", hsv)
            cv2.imshow("Mask", mask)
            cv2.imshow("Masked Frame", masked_frame)

            # Tunggu 1 ms dan periksa apakah tombol 'q' ditekan untuk keluar
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Node dihentikan.")

    finally:
        # Hentikan kamera dan tutup semua window
        vs.stop()
        cv2.destroyAllWindows()