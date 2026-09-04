#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tkinter as tk
from tkinter import Button

class CameraSubscriberGUI:
    def __init__(self, root):
        # Inisialisasi ROS node
        rospy.init_node('camera_subscriber_gui', anonymous=False)

        # Inisialisasi cv_bridge
        self.bridge = CvBridge()

        # Variabel untuk efek Canny
        self.apply_canny = False

        # Buat GUI
        self.root = root
        self.root.title("ROS Camera Subscriber with Canny Effect")

        # Button untuk mengaktifkan/menonaktifkan Canny
        self.canny_button = Button(root, text="Toggle Canny", command=self.toggle_canny)
        self.canny_button.pack()

        # Subscribe ke topik kamera
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

    def toggle_canny(self):
        # Toggle efek Canny
        self.apply_canny = not self.apply_canny
        rospy.loginfo(f"Canny Effect: {'ON' if self.apply_canny else 'OFF'}")

    def image_callback(self, msg):
        try:
            # Konversi gambar ROS ke OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Terapkan efek Canny jika diaktifkan
            if self.apply_canny:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                edges = cv2.Canny(gray, 100, 200)  # Parameter threshold bisa disesuaikan
                frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

            # Tampilkan gambar di jendela OpenCV
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error dalam callback: {e}")

    def run(self):
        # Jalankan main loop Tkinter
        self.root.mainloop()

if __name__ == '__main__':
    # Buat jendela Tkinter
    root = tk.Tk()

    # Inisialisasi dan jalankan node
    node = CameraSubscriberGUI(root)
    node.run()

    # Tutup jendela OpenCV saat program berhenti
    cv2.destroyAllWindows()