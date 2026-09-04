import cv2
import numpy as np

class ColorSegment:
    def __init__(self, table_name):
        # Membaca tabel warna dari file biner
        self.table_ = np.fromfile(table_name, dtype=np.uint8)
        self.table_ = self.table_.reshape((64, 64, 64))  # Mengubah bentuk tabel menjadi 64x64x64

    def segment(self, image, roi):
        if image.shape[0] == 0 or image.shape[1] == 0:
            return False
        elif image.shape[0] != self.segment_result_.shape[0] or image.shape[1] != self.segment_result_.shape[1]:
            self.segment_result_ = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        if len(image.shape) < 3 or image.shape[2] < 3:
            return False

        x_min, y_min = roi[0], roi[1]
        x_max, y_max = roi[0] + roi[2], roi[1] + roi[3]

        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                if y_min <= i <= y_max and x_min <= j <= x_max:
                    b, g, r = image[i, j]
                    self.segment_result_[i, j] = self.table_[b // 4, g // 4, r // 4]
                else:
                    self.segment_result_[i, j] = 0  # Area di luar ROI di-set ke 0

        return True

# Contoh penggunaan
if __name__ == "__main__":
    color_segment = ColorSegment("color_table.bin")
    image = cv2.imread("input_image.jpg")
    roi = (50, 50, 200, 200)  # Contoh ROI (x, y, width, height)
    if color_segment.segment(image, roi):
        cv2.imshow("Segmented Image", color_segment.segment_result_)
        cv2.waitKey(0)
        cv2.destroyAllWindows()