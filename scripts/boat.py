import cv2
import numpy as np
MIN_RED = 100
MIN_VER = 100

cv_image = cv2.imread('fase.png')
hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
rows, cols, a = cv_image.shape
print(hsv)

########## DETECTA LARANJA  ################################################
#Amarelo 28 105 222
#Verde low: 38 78 187  high: 42 86 201
#branco: 0 0 201 1 1 255
lowerblaranja = np.array([38, 78, 187])
upperblaranja = np.array([42, 86, 201])
mask_laranja = cv2.inRange(hsv, lowerblaranja, upperblaranja)    

rows_lar, cols_lar = mask_laranja.shape
cv2.imshow('mask_laranja', mask_laranja)
cv2.waitKey(3)


lowerbbranco = np.array([0, 0, 201])
upperbbranco = np.array([4, 4, 255])
mask_branco = cv2.inRange(hsv, lowerbbranco, upperbbranco)    

rows_lar, cols_lar = mask_branco.shape
cv2.imshow('mask_branco', mask_branco)
cv2.waitKey(0)


