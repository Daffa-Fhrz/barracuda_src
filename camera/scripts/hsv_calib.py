import cv2
import numpy as np


LEBAR_FRAME = 640
TINGGI_FRAME = 480
FPS_FRAME = 30

# Fungsi kosong untuk trackbar
def nothing(x):
    pass

# Membuat jendela untuk trackbar
cv2.namedWindow('HSV Color Picker')

# Membuat trackbar untuk lower dan upper HSV
cv2.createTrackbar('Lower H', 'HSV Color Picker', 0, 179, nothing)
cv2.createTrackbar('Lower S', 'HSV Color Picker', 0, 255, nothing)
cv2.createTrackbar('Lower V', 'HSV Color Picker', 0, 255, nothing)
cv2.createTrackbar('Upper H', 'HSV Color Picker', 179, 179, nothing)
cv2.createTrackbar('Upper S', 'HSV Color Picker', 255, 255, nothing)
cv2.createTrackbar('Upper V', 'HSV Color Picker', 255, 255, nothing)

# Membaca gambar atau video (ganti dengan path gambar/video Anda)
cap = cv2.VideoCapture(0)  # Menggunakan webcam, atau ganti dengan path file video/gambar

cap.set(cv2.CAP_PROP_FRAME_WIDTH, LEBAR_FRAME)   
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TINGGI_FRAME) 
cap.set(cv2.CAP_PROP_FPS, FPS_FRAME)         

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)

while True:
    # Membaca frame dari kamera
    ret, frame = cap.read()
    if not ret:
        break

    # Mengubah frame ke HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Mendapatkan nilai lower dan upper dari trackbar
    lower_h = cv2.getTrackbarPos('Lower H', 'HSV Color Picker')
    lower_s = cv2.getTrackbarPos('Lower S', 'HSV Color Picker')
    lower_v = cv2.getTrackbarPos('Lower V', 'HSV Color Picker')
    upper_h = cv2.getTrackbarPos('Upper H', 'HSV Color Picker')
    upper_s = cv2.getTrackbarPos('Upper S', 'HSV Color Picker')
    upper_v = cv2.getTrackbarPos('Upper V', 'HSV Color Picker')

    # Membuat array lower dan upper HSV
    lower_hsv = np.array([lower_h, lower_s, lower_v])
    upper_hsv = np.array([upper_h, upper_s, upper_v])

    # Membuat mask berdasarkan range HSV
    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

    # Mengaplikasikan mask ke frame asli
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Menampilkan frame asli, mask, dan hasil masking
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    # Keluar dari loop jika tombol 'q' ditekan
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Melepaskan kamera dan menutup semua jendela
cap.release()
cv2.destroyAllWindows()