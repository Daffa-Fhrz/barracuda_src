#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import cv2
import numpy as np
import random

# Parameter Particle Filter
NUM_PARTICLES = 50
NOISE_STD = 10
RESAMPLE_THRESHOLD = 0.5

# Fungsi untuk menghasilkan partikel awal
def initialize_particles(image):
    h, w = image.shape[:2]
    particles = np.zeros((NUM_PARTICLES, 2), dtype=np.float32)
    particles[:, 0] = np.random.uniform(0, w, NUM_PARTICLES)  # x
    particles[:, 1] = np.random.uniform(0, h, NUM_PARTICLES)  # y
    return particles

# Fungsi untuk memprediksi posisi partikel
def predict(particles):
    particles += np.random.normal(0, NOISE_STD, particles.shape)
    return particles

# Fungsi untuk menghitung bobot partikel
def calculate_weights(image, particles, target_color):
    h, w = image.shape[:2]
    weights = np.zeros(NUM_PARTICLES)
    for i, (x, y) in enumerate(particles):
        if 0 <= x < w and 0 <= y < h:
            color = image[int(y), int(x)]
            distance = np.linalg.norm(color - target_color)
            weights[i] = 1.0 / (distance + 1e-6)
        else:
            weights[i] = 0.0
    weights /= np.sum(weights)
    return weights

# Fungsi untuk resampling partikel
def resample(particles, weights):
    indices = np.random.choice(np.arange(NUM_PARTICLES), size=NUM_PARTICLES, p=weights)
    return particles[indices]

# Fungsi untuk membuat marker di RViz
def create_marker(x, y):
    marker = Marker()
    marker.header.frame_id = "map"  # Frame referensi
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0  # Opacity
    marker.color.r = 1.0  # Warna merah
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    return marker

# Fungsi utama
def main():
    rospy.init_node('particle_filter_ros', anonymous=True)
    pub = rospy.Publisher('/estimated_position', Point, queue_size=10)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Error: Could not open camera.")
        return

    # Set resolusi
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Target warna (BGR format)
    target_color = np.array([0, 165, 255])  # Warna oranye

    # Inisialisasi partikel
    ret, frame = cap.read()
    particles = initialize_particles(frame)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # Resize frame untuk mempercepat proses
        frame = cv2.resize(frame, (320, 240))

        # Prediksi posisi partikel
        particles = predict(particles)

        # Hitung bobot partikel
        weights = calculate_weights(frame, particles, target_color)

        # Resampling partikel
        if np.max(weights) < RESAMPLE_THRESHOLD:
            particles = resample(particles, weights)

        # Estimasi posisi objek
        estimated_position = np.average(particles, axis=0, weights=weights)

        # Kirim posisi estimasi ke ROS
        point_msg = Point()
        point_msg.x = estimated_position[0]
        point_msg.y = estimated_position[1]
        point_msg.z = 0  # 2D, jadi z = 0
        pub.publish(point_msg)

        # Buat marker untuk RViz
        marker = create_marker(estimated_position[0], estimated_position[1])
        marker_pub.publish(marker)

        # Tampilkan hasil
        result_frame = frame.copy()
        for (x, y) in particles:
            cv2.circle(result_frame, (int(x), int(y)), 2, (0, 255, 0), -1)
        cv2.circle(result_frame, (int(estimated_position[0]), int(estimated_position[1])), 10, (0, 0, 255), -1)
        cv2.imshow("Particle Filter Result", result_frame)

        # Keluar jika tombol 'q' ditekan
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass