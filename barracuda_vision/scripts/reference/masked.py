import cv2
import numpy as np

# Fungsi kosong untuk callback trackbar
def nothing(x):
    pass

# Inisialisasi webcam
cap = cv2.VideoCapture(0)

# Buat jendela untuk menampilkan hasil
cv2.namedWindow('Masked Frame')

# Buat trackbar untuk mengatur nilai HSV lower dan upper bounds
cv2.createTrackbar('H Lower', 'Masked Frame', 0, 179, nothing)
cv2.createTrackbar('S Lower', 'Masked Frame', 0, 255, nothing)
cv2.createTrackbar('V Lower', 'Masked Frame', 0, 255, nothing)
cv2.createTrackbar('H Upper', 'Masked Frame', 179, 179, nothing)
cv2.createTrackbar('S Upper', 'Masked Frame', 255, 255, nothing)
cv2.createTrackbar('V Upper', 'Masked Frame', 255, 255, nothing)

while True:
    # Baca frame dari webcam
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Ubah frame ke ruang warna HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Ambil nilai trackbar untuk lower dan upper bounds
    h_lower = cv2.getTrackbarPos('H Lower', 'Masked Frame')
    s_lower = cv2.getTrackbarPos('S Lower', 'Masked Frame')
    v_lower = cv2.getTrackbarPos('V Lower', 'Masked Frame')
    h_upper = cv2.getTrackbarPos('H Upper', 'Masked Frame')
    s_upper = cv2.getTrackbarPos('S Upper', 'Masked Frame')
    v_upper = cv2.getTrackbarPos('V Upper', 'Masked Frame')

    # Tentukan rentang warna HSV berdasarkan trackbar
    lower_bound = np.array([h_lower, s_lower, v_lower])
    upper_bound = np.array([h_upper, s_upper, v_upper])

    # Buat mask berdasarkan rentang warna
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Aplikasikan mask ke frame asli
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

    # Tampilkan frame asli dan frame yang sudah di-mask
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Masked Frame', masked_frame)

    # Tekan 'q' untuk keluar dari loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Lepaskan webcam dan tutup semua jendela
cap.release()
cv2.destroyAllWindows()