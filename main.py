# install OpenCv and numpy
# pip install opencv-python
# pip install numpy

import cv2
import numpy as np
import time

## For saving the video
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('result.avi', fourcc, 20.0, (640, 480))

cap = cv2.VideoCapture(0)

# Pause the cature for 3 second for saving the background
time.sleep(3)
background = 0

for i in range(30):
    ret, background = cap.read()

# mirror the background
background = np.flip(background, axis=1)

while (cap.isOpened()):
    ret, img = cap.read()

    # Flipping the image
    img = np.flip(img, axis=1)

    # Converting image to HSV color space as HSV are easy to manage.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    value = (35, 35)

    blurred = cv2.GaussianBlur(hsv, value, 0)

    # Detect the lower range of red color
    lower_red = np.array([0, 120, 90])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    # Detect the upper range of red color
    lower_red = np.array([170, 120, 90])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # Addition of the two masks.
    mask = mask1 + mask2
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    # Search for the red color and replace with red background
    #img[np.where(mask != 255)] = 0
    img[np.where(mask == 255)] = background[np.where(mask == 255)]

    #Save the output file
    out.write(img)

    cv2.imshow('Invisible cloack', img)
    k = cv2.waitKey(10)
    if k == 27:
        break
