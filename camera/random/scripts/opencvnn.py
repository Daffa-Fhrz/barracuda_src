import os
import cv2
import numpy as np
import matplotlib.pyplot as mp 


#img = cv2.imread('ball2.png',cv2.IMREAD_COLOR)
#imgpt = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
image = cv2.imread('omni2.jpg',cv2.IMREAD_COLOR)
image2 = cv2.imread('omni2.jpg',cv2.IMREAD_GRAYSCALE)
imageAbu = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
imageAbu2 = cv2.cvtColor(imageRGB,cv2.COLOR_RGB2GRAY)

b, g, r = cv2.split(image)

imageMask = image2.copy()
maskRec = cv2.rectangle(imageMask,(0,0),(640,480),(0,0,0),thickness= -1, lineType= 1)

# Show the channels
mp.imshow(maskRec, cmap=None)

mp.show()
#TRY MATPOTLIB 
"""
dimension = img.shape
grayimage = cv2.imread('ball1.png',cv2.IMREAD_GRAYSCALE)
print(img)
mp.imshow(imgcb)
mp.show()
"""
#TRY RESIZE AND FLIP
"""
image_cb =  imgpt.copy()
image_cb[2,0] = 255
image_cb[2,3] = 255
image_cb[3,2] = 255
image_cb[5,4] = 255
mp.imshow(imgpt)
mp.show()
crop_img = imgpt[150:300, 200:350]
mp.imshow(crop_img)
mp.show()
flip_img = cv2.flip(imgpt, -1)
mp.imshow(flip_img)
mp.show()

garis = imgpt.copy()

cv2.line(garis,(200,100),(400,100),(0,0,255), thickness=2, lineType=cv2.LINE_AA)
imkotak = cv2.rectangle(imgpt,(207,187 ),(326,287),(255, 0, 255),thickness=-3,lineType=cv2.LINE_8 )
text = "Ball"
fontScale = 1
fontFace = cv2.FONT_HERSHEY_PLAIN
fontColor = (0, 255, 0)
fontThickness = 2
cv2.putText(imkotak, text, (205, 179), fontFace, fontScale, fontColor, fontThickness, cv2.LINE_AA)
mp.imshow(imkotak)
mp.show()
"""





