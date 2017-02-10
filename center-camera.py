#!/usr/bin/python

import cv2 
import sys
import numpy as np

window_name = "center camera"
cv2.namedWindow(window_name)
camera_stream = cv2.VideoCapture(0)

success, image = camera_stream.read()
if not success:
    sys.exit()

height, width, channels = image.shape
center_x = width / 2
center_y = height / 2
print "Image center (%s, %s)" % (center_x, center_y)

target_x = 491
target_y = 255
print "Calculated target location (%s, %s)" % (target_x, target_y)

show_target = True
success = True
while success:
    success, image = camera_stream.read()
    if success:
        cv2.circle(image, (center_x, center_y), 10, (255, 0, 0), -1)
        #cv2.circle(image, (201, 375), 10, (0, 255, 0), -1)
        #cv2.circle(image, (192, 255), 10, (0, 255, 0), -1)
        #cv2.circle(image, ( 42, 255), 10, (0, 255, 0), -1)
        #cv2.circle(image, (640, 255), 10, (0, 255, 0), -1)
        cv2.imshow(window_name, image)
        cv2.waitKey(1)
        #cv2.imwrite('2D-to-3D-with-camera-rotation.jpg', image)
        #success = False
