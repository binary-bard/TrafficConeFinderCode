#!/usr/bin/env python
#
#MIT License

#Copyright (c) 2016 Saurabh Khanduja

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

delay = 200
if (len(sys.argv) == 3):
  delay = sys.argv[2]

cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('HMin1','image',0,179,nothing) # Hue is from 0-179 for Opencv
cv2.createTrackbar('HMax1','image',0,179,nothing)
cv2.createTrackbar('HMin2','image',0,179,nothing)
cv2.createTrackbar('HMax2','image',0,179,nothing)
cv2.createTrackbar('SMin','image',0,255,nothing)
cv2.createTrackbar('SMax','image',0,255,nothing)
cv2.createTrackbar('VMin','image',0,255,nothing)
cv2.createTrackbar('VMax','image',0,255,nothing)

# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('HMax1', 'image', 19)
cv2.setTrackbarPos('HMin2', 'image', 160)
cv2.setTrackbarPos('HMax2', 'image', 179)
cv2.setTrackbarPos('SMin', 'image', 135)
cv2.setTrackbarPos('VMin', 'image', 135)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize to check if HSV min/max value changes
hMin1 = hMin2 = sMin = vMin = hMax1 = hMax2 = sMax = vMax = 0

cap = cv2.VideoCapture(sys.argv[1])
if(cap.isOpened() == False):
    print("Error: Could not open video file " + sys.argv[1] + ". Using default device.")
    cap = cv2.VideoCapture(0)

ret = cap.isOpened()
while(ret):
    ret, frame = cap.read()
    img = cv2.resize(frame, (480, 360))
    
    # get current positions of all trackbars
    hMin1 = cv2.getTrackbarPos('HMin1','image')
    hMin2 = cv2.getTrackbarPos('HMin2','image')
    sMin = cv2.getTrackbarPos('SMin','image')
    vMin = cv2.getTrackbarPos('VMin','image')

    hMax1 = cv2.getTrackbarPos('HMax1','image')
    hMax2 = cv2.getTrackbarPos('HMax2','image')
    sMax = cv2.getTrackbarPos('SMax','image')
    vMax = cv2.getTrackbarPos('VMax','image')

    # Set minimum and max HSV values to display
    lower1 = np.array([hMin1, sMin, vMin])
    upper1 = np.array([hMax1, sMax, vMax])
    lower2 = np.array([hMin2, sMin, vMin])
    upper2 = np.array([hMax2, sMax, vMax])

    # Create HSV Image and threshold into a range.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    
    mask = mask1 + mask2
    output = cv2.bitwise_and(img, img, mask = mask1)

    # Display output image
    cv2.imshow('image',output)

    # Wait for a second for each frame
    k = cv2.waitKey(delay) & 0xFF
    if k == 27:
        break
        
cv2.destroyAllWindows()
