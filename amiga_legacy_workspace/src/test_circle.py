import numpy as np
import cv2
import os

with open('/home/yilong/Desktop/amiga_extra/rgb.npy', 'rb') as f:
    rgb = np.load(f)

output = rgb.copy()
#gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
filename = 'rgb.jpg'
  
# Using cv2.imwrite() method
# Saving the image
cv2.imwrite(filename, rgb)

cv2.imshow("input", rgb)
cv2.waitKey(0)
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)
# ensure at least some circles were found
if circles is not None:
	# convert the (x, y) coordinates and radius of the circles to integers
	circles = np.round(circles[0, :]).astype("int")
	# loop over the (x, y) coordinates and radius of the circles
	for (x, y, r) in circles:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
		cv2.circle(output, (x, y), r, (0, 255, 0), 4)
		cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
	# show the output image
	cv2.imshow("output", np.hstack([image, output]))
	cv2.waitKey(0)
