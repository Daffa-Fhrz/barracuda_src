import cv2

# Membuka kamera (0 biasanya adalah kamera default)
cap = cv2.VideoCapture(2)

# Periksa apakah kamera berhasil dibuka
if not cap.isOpened():
    print("Error: Tidak dapat membuka kamera.")
    exit()

# Loop untuk membaca frame dari kamera
while True:
    # Membaca frame
    ret, frame = cap.read()

    # Jika frame berhasil dibaca, ret akan bernilai True
    if not ret:
        print("Error: Tidak dapat membaca frame.")
        break

    # Menampilkan frame
    cv2.imshow('Kamera', frame)

    # Tekan 'q' untuk keluar dari loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Melepaskan kamera dan menutup semua jendela
cap.release()
cv2.destroyAllWindows()