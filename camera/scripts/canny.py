import cv2
import numpy as np

# Fungsi untuk memproses frame dan mendeteksi garis
def detect_lines(frame):
    # Konversi ke grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Mengaplikasikan Gaussian Blur untuk mengurangi noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Thresholding untuk mengisolasi warna putih
    _, white_mask = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

    # Mengaplikasikan Canny Edge Detection pada mask
    edges = cv2.Canny(white_mask, 50, 150)

    # Menggunakan Hough Line Transform untuk mendeteksi garis
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=50,
                            minLineLength=50, maxLineGap=10)

    # Gambar garis yang terdeteksi pada frame asli
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Gambar garis hijau

    return frame, white_mask, edges, lines

# Membuka kamera (0 adalah indeks kamera default)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Tidak dapat membuka kamera.")
    exit()

while True:
    # Membaca frame dari kamera
    ret, frame = cap.read()

    if not ret:
        print("Error: Tidak dapat membaca frame.")
        break

    # Memproses frame untuk mendeteksi garis
    frame, white_mask, edges, lines = detect_lines(frame)

    # Menampilkan frame hasil deteksi dan masking
    cv2.imshow('Detected Lines', frame)
    cv2.imshow('White Mask', white_mask)

    # Tekan tombol 'q' untuk keluar dari loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Melepaskan kamera dan menutup semua jendela
cap.release()
cv2.destroyAllWindows()