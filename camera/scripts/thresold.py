import cv2
import numpy as np

# Fungsi callback untuk trackbar (tidak melakukan apa-apa)
def nothing(x):
    pass

# Membuka kamera (0 adalah indeks kamera default)
cap = cv2.VideoCapture(0)

# Jika menggunakan file video, ganti dengan path file video
# cap = cv2.VideoCapture('video.mp4')

# Periksa apakah kamera/video berhasil dibuka
if not cap.isOpened():
    print("Error: Tidak dapat membuka kamera/video.")
    exit()

# Membuat window untuk menampilkan hasil
cv2.namedWindow('Thresholding')

# Membuat trackbar untuk mengatur nilai threshold
cv2.createTrackbar('Threshold', 'Thresholding', 127, 255, nothing)

while True:
    # Membaca frame dari kamera/video
    ret, frame = cap.read()

    if not ret:
        print("Error: Tidak dapat membaca frame.")
        break

    # Mengonversi frame ke grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Mendapatkan nilai threshold dari trackbar
    threshold_value = cv2.getTrackbarPos('Threshold', 'Thresholding')

    # Mengaplikasikan thresholding
    _, thresholded = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)

    # Menampilkan frame asli dan hasil thresholding
    cv2.imshow('Original Video', frame)
    cv2.imshow('Thresholding', thresholded)

    # Tekan tombol 'q' untuk keluar dari loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Melepaskan kamera/video dan menutup semua jendela
cap.release()
cv2.destroyAllWindows()