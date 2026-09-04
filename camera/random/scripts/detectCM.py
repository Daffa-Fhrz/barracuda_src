import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

# Fungsi untuk menghitung jarak
def calculate_distance(focal_length, real_size, pixel_size):
    return (focal_length * real_size) / pixel_size

# Fungsi untuk memperbarui focal length dari slider
def update_focal_length(value):
    global f_x
    f_x = float(value)
    label_focal_length.config(text=f"Focal Length: {f_x:.2f}")

# Fungsi untuk menangani frame dari kamera
def update_frame():
    ret, frame = cap.read()
    if ret:
        # Konversi ke HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Range warna oranye dalam HSV
        lower_orange = np.array([10, 100, 100])
        upper_orange = np.array([25, 255, 255])

        # Masking untuk mendapatkan area oranye
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Operasi morfologi untuk menghilangkan noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Cari kontur
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Jika ditemukan kontur
        if len(contours) > 0:
            # Ambil kontur terbesar (asumsi itu adalah bola)
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

            # Gambar lingkaran di sekitar objek
            if radius > 10:  # Filter kontur kecil
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                # Hitung jarak menggunakan f_x
                distance = calculate_distance(f_x, real_radius, radius * 2)  # radius * 2 = diameter

                # Tampilkan teks jarak di atas objek
                text = f"Distance: {distance:.2f} cm"
                cv2.putText(frame, text, (int(x) - 50, int(y) - int(radius) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Konversi frame OpenCV ke format yang bisa ditampilkan di Tkinter
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image=img)
        label_video.imgtk = imgtk
        label_video.config(image=imgtk)

    # Panggil fungsi ini lagi setelah 10 ms
    label_video.after(10, update_frame)

# Inisialisasi Tkinter
root = tk.Tk()
root.title("Deteksi Bola dengan Kontur dan Slider Focal Length")

# Inisialisasi kamera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Focal length awal
f_x = 458.9  # Nilai awal focal length
real_radius = 7.5  # Ukuran nyata objek (dalam cm)

# Label untuk menampilkan video
label_video = tk.Label(root)
label_video.pack()

# Slider untuk mengatur focal length
slider_focal_length = ttk.Scale(
    root,
    from_=100,  # Nilai minimum focal length
    to=1000,    # Nilai maksimum focal length
    value=f_x,  # Nilai awal
    orient=tk.HORIZONTAL,
    command=update_focal_length
)
slider_focal_length.pack(pady=10)

# Label untuk menampilkan nilai focal length
label_focal_length = tk.Label(root, text=f"Focal Length: {f_x:.2f}")
label_focal_length.pack()

# Mulai memperbarui frame
update_frame()

# Jalankan aplikasi Tkinter
root.mainloop()

# Setelah aplikasi ditutup, lepaskan kamera
cap.release()