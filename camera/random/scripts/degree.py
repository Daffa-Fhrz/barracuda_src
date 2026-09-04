import cv2
import numpy as np
import math

# Fungsi untuk menghitung sudut (derajat) antara titik tengah dan posisi objek
def calculate_angle(center_x, center_y, obj_x, obj_y):
    # Hitung perbedaan posisi
    dx = obj_x - center_x
    dy = center_y - obj_y  # Dibalik karena sumbu Y di OpenCV terbalik

    # Hitung sudut dalam radian
    angle_rad = math.atan2(dy, dx)

    # Konversi ke derajat
    angle_deg = math.degrees(angle_rad)

    # Pastikan sudut positif (0-360 derajat)
    if angle_deg < 0:
        angle_deg += 360

    return angle_deg

# Fungsi untuk menentukan kuadran
def get_quadrant(angle):
    if 0 <= angle < 90:
        return "Quadran 1"
    elif 90 <= angle < 180:
        return "Quadran 2"
    elif 180 <= angle < 270:
        return "Quadran 3"
    else:
        return "Quadran 4"

# Inisialisasi kamera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Titik tengah frame
center_x = 320  # 640 / 2
center_y = 240  # 480 / 2

while True:
    ret, frame = cap.read()
    if not ret:
        print("Gagal membaca frame dari kamera. Pastikan kamera terhubung.")
        break

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

            # Gambar titik pusat bola
            cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), 3)

            # Hitung sudut posisi bola
            angle = calculate_angle(center_x, center_y, int(x), int(y))
            quadrant = get_quadrant(angle)

            # Gambar garis dari titik tengah ke posisi bola
            cv2.line(frame, (center_x, center_y), (int(x), int(y)), (255, 0, 0), 2)

            # Tampilkan teks sudut dan kuadran
            text_angle = f"Angle: {angle:.2f} deg"
            text_quadrant = f"Quadrant: {quadrant}"
            cv2.putText(frame, text_angle, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, text_quadrant, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Gambar garis tengah horizontal dan vertikal
    cv2.line(frame, (center_x, 0), (center_x, 480), (255, 255, 255), 1)  # Garis vertikal
    cv2.line(frame, (0, center_y), (640, center_y), (255, 255, 255), 1)  # Garis horizontal

    # Tampilkan frame
    cv2.imshow("Deteksi Bola dengan Sudut dan Kuadran", frame)

    # Keluar dengan menekan 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Setelah aplikasi ditutup, lepaskan kamera
cap.release()
cv2.destroyAllWindows()