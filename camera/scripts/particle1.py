import cv2
import numpy as np
import random

# Parameter Particle Filter
NUM_PARTICLES = 100  # Jumlah partikel
NOISE_STD = 10       # Standar deviasi noise untuk pergerakan partikel

# Parameter Kalibrasi Kamera (ganti dengan nilai kalibrasi Anda)
camera_matrix = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([-0.4, 0.1, 0, 0], dtype=np.float32)  # Contoh koefisien distorsi

# Inisialisasi partikel secara acak di seluruh frame
def initialize_particles(frame_shape):
    particles = []
    for _ in range(NUM_PARTICLES):
        x = random.randint(0, frame_shape[1] - 1)  # Posisi x acak
        y = random.randint(0, frame_shape[0] - 1)  # Posisi y acak
        particles.append((x, y))
    return particles

# Update posisi partikel dengan menambahkan noise
def update_particles(particles):
    updated_particles = []
    for (x, y) in particles:
        x += int(random.gauss(0, NOISE_STD))  # Tambahkan noise pada x
        y += int(random.gauss(0, NOISE_STD))  # Tambahkan noise pada y
        updated_particles.append((x, y))
    return updated_particles

# Hitung bobot partikel berdasarkan hasil thresholding
def calculate_weights(particles, thresholded):
    weights = []
    for (x, y) in particles:
        if 0 <= x < thresholded.shape[1] and 0 <= y < thresholded.shape[0]:
            weights.append(thresholded[y, x] / 255.0)  # Bobot berdasarkan intensitas putih
        else:
            weights.append(0.0)  # Bobot 0 jika partikel di luar frame
    return weights

# Resampling partikel berdasarkan bobot
def resample_particles(particles, weights):
    new_particles = random.choices(particles, weights=weights, k=NUM_PARTICLES)
    return new_particles

# Fungsi utama
def main():
    # Membuka kamera (0 adalah indeks kamera default)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Tidak dapat membuka kamera.")
        exit()

    # Membaca frame pertama untuk inisialisasi
    ret, frame = cap.read()
    if not ret:
        print("Error: Tidak dapat membaca frame.")
        exit()

    # Inisialisasi partikel
    particles = initialize_particles(frame.shape)

    while True:
        # Membaca frame dari kamera
        ret, frame = cap.read()
        if not ret:
            print("Error: Tidak dapat membaca frame.")
            break

        # Undistort frame untuk menghilangkan distorsi lensa
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # Mengonversi frame ke grayscale
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)

        # Thresholding untuk mengisolasi warna putih
        _, thresholded = cv2.threshold(gray, 114, 255, cv2.THRESH_BINARY)

        # Update partikel
        particles = update_particles(particles)

        # Hitung bobot partikel
        weights = calculate_weights(particles, thresholded)

        # Resampling partikel
        particles = resample_particles(particles, weights)

        # Gambar partikel pada frame
        for (x, y) in particles:
            cv2.circle(undistorted_frame, (x, y), 2, (0, 255, 0), -1)  # Gambar partikel sebagai titik hijau

        # Menampilkan frame asli dan hasil thresholding
        cv2.imshow('Original Video', undistorted_frame)
        cv2.imshow('Thresholding', thresholded)

        # Tekan tombol 'q' untuk keluar dari loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Melepaskan kamera dan menutup semua jendela
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()