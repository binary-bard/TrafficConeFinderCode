#!/usr/bin/env python
#
#MIT License

#Original code:
#Copyright (c) 2016 Saurabh Khanduja

#Changes
#Copyright (c) 2017 Ajay Guleria

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

import cv2
import sys
import numpy as np

def nothing(x):
    pass

# Check if filename is passed
if (len(sys.argv) <= 1 or len(sys.argv) > 3) :
    print("Usage: hsvThresholder.py <VideoFile> [delay_in_ms]")
    exit()

delay = 20
if (len(sys.argv) == 3):
  delay = int(sys.argv[2])

cv2.namedWindow('ctrl')
# create trackbars for color change
# Hue is from 0-179 for Opencv, HMax < HMin inverts the range
cv2.createTrackbar('HMin','ctrl',0,179,nothing)
cv2.createTrackbar('HMax','ctrl',0,179,nothing)
cv2.createTrackbar('SMin','ctrl',0,255,nothing)
cv2.createTrackbar('SMax','ctrl',0,255,nothing)
cv2.createTrackbar('VMin','ctrl',0,255,nothing)
cv2.createTrackbar('VMax','ctrl',0,255,nothing)

# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('HMax', 'ctrl', 19)
cv2.setTrackbarPos('HMin', 'ctrl', 160)
cv2.setTrackbarPos('SMin', 'ctrl', 135)
cv2.setTrackbarPos('VMin', 'ctrl', 135)
cv2.setTrackbarPos('SMax', 'ctrl', 255)
cv2.setTrackbarPos('VMax', 'ctrl', 255)

# Initialize to check if HSV min/max value changes
hMin = sMin = vMin = hMax = sMax = vMax = 0

cap = cv2.VideoCapture(sys.argv[1])
if(cap.isOpened() == False):
    print("Error: Could not open video file " + sys.argv[1] + ". Using default device.")
    cap = cv2.VideoCapture(0)

cv2.namedWindow('image')
ret = cap.isOpened()
while(ret):
    ret, frame = cap.read()
    img = cv2.resize(frame, (640, 480))
    # Create HSV Image and threshold into a range.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin','ctrl')
    sMin = cv2.getTrackbarPos('SMin','ctrl')
    vMin = cv2.getTrackbarPos('VMin','ctrl')

    hMax = cv2.getTrackbarPos('HMax','ctrl')
    sMax = cv2.getTrackbarPos('SMax','ctrl')
    vMax = cv2.getTrackbarPos('VMax','ctrl')

    # Set minimum and max HSV values to display
    if(hMin > hMax):
        lower1 = np.array([0, sMin, vMin])
        upper1 = np.array([hMax, sMax, vMax])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        lower2 = np.array([hMin, sMin, vMin])
        upper2 = np.array([255, sMax, vMax])
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])
        mask = cv2.inRange(hsv, lower, upper)
        
    output = cv2.bitwise_and(img, img, mask=mask)

    # Display output image
    cv2.imshow('image',output)

    # Wait for a second for each frame
    k = cv2.waitKey(delay) & 0xFF
    if k == 27:
        break
        
cv2.destroyAllWindows()
