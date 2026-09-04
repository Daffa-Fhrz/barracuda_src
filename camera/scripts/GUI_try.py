#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tkinter as tk
from tkinter import Button, Label, Entry, Frame
import yaml
import os
import rospkg  # Untuk mendapatkan path ke package

class CameraSubscriberGUI:
    def __init__(self, root):
        # Inisialisasi ROS node
        rospy.init_node('camera_subscriber_gui_advanced', anonymous=True)

        # Inisialisasi cv_bridge
        self.bridge = CvBridge()

        # Variabel untuk efek dan parameter
        self.apply_canny = False
        self.canny_threshold1 = 100
        self.canny_threshold2 = 200

        # Dapatkan path ke direktori config di package camera
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('camera')  # Ganti 'camera' dengan nama package Anda
        self.config_dir = f"{package_path}/config"
        os.makedirs(self.config_dir, exist_ok=True)  # Buat direktori jika belum ada

        # Buat GUI
        self.root = root
        self.root.title("ROS Camera Subscriber with Advanced GUI")

        # Frame untuk tombol efek
        self.left_frame = Frame(root)
        self.left_frame.pack(side=tk.LEFT, padx=10, pady=10)

        # Frame untuk output gambar
        self.right_frame = Frame(root)
        self.right_frame.pack(side=tk.RIGHT, padx=10, pady=10)

        # Frame untuk input parameter
        self.bottom_frame = Frame(root)
        self.bottom_frame.pack(side=tk.BOTTOM, padx=10, pady=10)

        # Button untuk mengaktifkan/menonaktifkan Canny
        self.canny_button = Button(self.left_frame, text="Toggle Canny", command=self.toggle_canny)
        self.canny_button.pack(pady=5)

        # Label dan Entry untuk threshold Canny
        self.threshold1_label = Label(self.bottom_frame, text="Canny Threshold 1:")
        self.threshold1_label.pack()
        self.threshold1_entry = Entry(self.bottom_frame)
        self.threshold1_entry.insert(0, str(self.canny_threshold1))
        self.threshold1_entry.pack()

        self.threshold2_label = Label(self.bottom_frame, text="Canny Threshold 2:")
        self.threshold2_label.pack()
        self.threshold2_entry = Entry(self.bottom_frame)
        self.threshold2_entry.insert(0, str(self.canny_threshold2))
        self.threshold2_entry.pack()

        # Button untuk menyimpan parameter
        self.save_button = Button(self.bottom_frame, text="Save Parameters", command=self.save_parameters)
        self.save_button.pack(pady=10)

        # Subscribe ke topik kamera
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

    def toggle_canny(self):
        # Toggle efek Canny
        self.apply_canny = not self.apply_canny
        rospy.loginfo(f"Canny Effect: {'ON' if self.apply_canny else 'OFF'}")

    def save_parameters(self):
        # Simpan parameter ke file YAML di direktori config
        try:
            self.canny_threshold1 = int(self.threshold1_entry.get())
            self.canny_threshold2 = int(self.threshold2_entry.get())

            parameters = {
                'canny_threshold1': self.canny_threshold1,
                'canny_threshold2': self.canny_threshold2,
            }

            # Path ke file config.yaml di direktori config
            config_file = f"{self.config_dir}/config.yaml"

            with open(config_file, 'w') as file:
                yaml.dump(parameters, file)

            rospy.loginfo(f"Parameter berhasil disimpan ke {config_file}")
        except Exception as e:
            rospy.logerr(f"Gagal menyimpan parameter: {e}")

    def image_callback(self, msg):
        try:
            # Konversi gambar ROS ke OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Terapkan efek Canny jika diaktifkan
            if self.apply_canny:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                edges = cv2.Canny(gray, self.canny_threshold1, self.canny_threshold2)
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