import cv2
import numpy as np
import matplotlib.pyplot as pt
# read image
img = cv2.imread('omni2.jpg')
hh, ww = img.shape[:2]
hh2 = hh // 2
ww2 = ww // 2
# print(hh)
# print(ww)
# define circles
radius1 = 270
radius2 = 75
xc = hh // 2
yc = ww // 2 
center =  yc, xc
# Draw the circle


# draw filled circles in white on black background as masks
mask1 = np.zeros((hh,ww),dtype= np.uint8)
# cv2.circle(mask1, center, radius=50, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
# mask1 = cv2.circle(mask1, (xc,yc), radius1, (255,255,255), -1)
mask1 = cv2.circle(mask1, center, radius1, 255, -1)
mask2 = np.zeros_like(img)
mask2 = cv2.circle(mask2, (xc,yc), radius2, (255,255,255), -1)


# subtract masks and make into single channel
# mask = cv2.subtract(mask1)
hasil = cv2.bitwise_and(img, img, mask=mask1)
cv2.line(hasil, (320,0),(320,480),(255,0,0), thickness=1, lineType=cv2.LINE_AA)
cv2.line(hasil, (0,240),(640,240),(255,0,0), thickness=1, lineType=cv2.LINE_AA)
# put mask into alpha channel of input
# result = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
# result[:, :, 3] = mask[:,:,0]

# save result

# cv2.imshow('image', img)
# cv2.imshow('mask1', mask1)
# cv2.imshow('mask2', mask2)
# cv2.imshow('mask', mask)
# cv2.imshow('masked image', result)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
pt.imshow(cv2.cvtColor(hasil, cv2.COLOR_BGR2RGB))
pt.show()