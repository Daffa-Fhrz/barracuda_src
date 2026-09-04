import os
import cv2
import numpy as np
import matplotlib.pyplot as mp 

image = cv2.imread('omni2.jpg', cv2.IMREAD_COLOR)
img_rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
maskingImage = cv2.imread('OMNIMASK.png', cv2.IMREAD_GRAYSCALE)
retval, img_mask = cv2.threshold(maskingImage, 127, 255, cv2.THRESH_BINARY)
w_img = maskingImage.shape[0]
h_img = maskingImage.shape[1]

invertMask =cv2.bitwise_not(maskingImage)
imageCam = cv2.bitwise_and(img_rgb, img_rgb, mask=img_mask)
#print(maskingImage.shape)
mp.imshow(imageCam, cmap='gray')
mp.show()