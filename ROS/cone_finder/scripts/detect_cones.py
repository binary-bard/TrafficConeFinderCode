#!/usr/bin/env python

import numpy as np
import cv2

import os
import argparse
import glob

# Needed for publishing the messages
import rospy
import message_filters
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Pose2D
from cone_finder.msg import location_msgs as location_data
from cv_bridge import CvBridge, CvBridgeError
import threading

class Args(object):
    use_ros_topic = False
    debug = False
    fromMain = False
    image_dir = ''
    video_file = ''

args = Args()
pub = rospy.Publisher('cone_finder/locations', location_data, queue_size=10)
colorPub = rospy.Publisher("cone_finder/colorImage", Image, queue_size=10)
depthPub = rospy.Publisher("cone_finder/depthImage", Image, queue_size=10)
colorCIPub = rospy.Publisher("cone_finder/colorCamInfo", CameraInfo, queue_size=2)
depthCIPub = rospy.Publisher("cone_finder/depthCamInfo", CameraInfo, queue_size=2)

def is_cv2():
    # if we are using OpenCV 2, then our cv2.__version__ will start
    # with '2.'
    return check_opencv_version("2.")

def is_cv3():
    # if we are using OpenCV 3.X, then our cv2.__version__ will start
    # with '3.'
    return check_opencv_version("3.")

def check_opencv_version(major, lib=None):
    # if the supplied library is None, import OpenCV
    if lib is None:
        import cv2 as lib

    # return whether or not the current OpenCV version matches the
    # major version number
    return lib.__version__.startswith(major)

def convexHullIsPointingUp(hull):
    x, y, w, h = cv2.boundingRect(hull)

    aspectRatio = float(w) / h
    if aspectRatio > 0.9:
        return False

    listOfPointsAboveCenter = []
    listOfPointsBelowCenter = []

    intYcenter = y + h / 2

    # step through all points in convex hull
    for point in hull:
        # and add each point to
        # list of points above or below vertical center as applicable
        if point[0][1] < intYcenter:
            listOfPointsAboveCenter.append(point)

        if point[0][1] >= intYcenter:
            listOfPointsBelowCenter.append(point)

    intLeftMostPointBelowCenter = listOfPointsBelowCenter[0][0][0]
    intRightMostPointBelowCenter = listOfPointsBelowCenter[0][0][0]

    # determine left most point below center
    for point in listOfPointsBelowCenter:

            if point[0][0] < intLeftMostPointBelowCenter:
                intLeftMostPointBelowCenter = point[0][0]

        # determine right most point below center
    for point in listOfPointsBelowCenter:
        if point[0][0] >= intRightMostPointBelowCenter:
            intRightMostPointBelowCenter = point[0][0]

        # step through all points above center
    for point in listOfPointsAboveCenter:
        if point[0][0] < intLeftMostPointBelowCenter or \
         point[0][0] > intRightMostPointBelowCenter:
            return False

    # if we get here, shape has passed pointing up check
    return True

def find_cones(img, depthImg=None):
    h, w = img.shape[:2]
    image_centerX = w/2
    image_centerY = h  # y goes down from top

    # convert to HSV color space, this will produce better color filtering
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Threshold on low range of HSV red
    low_redl = np.array([0, 135, 135])
    low_redh = np.array([15, 255, 255])
    imgThreshLow = cv2.inRange(imgHSV, low_redl, low_redh)

    # threshold on high range of HSV red
    high_redl = np.array([159, 135, 135])
    high_redh = np.array([179, 255, 255])
    imgThreshHigh = cv2.inRange(imgHSV, high_redl, high_redh)

    # combine low range red thresh and high range red thresh
    imgThresh = cv2.bitwise_or(imgThreshLow, imgThreshHigh)

    # clone/copy thresh image before smoothing
    imgThreshSmoothed = imgThresh.copy()
    # open image (erode, then dilate)
    kernel = np.ones((3, 3), np.uint8)
    imgThreshSmoothed = cv2.erode(imgThresh, kernel, iterations=1)
    imgThreshSmoothed = cv2.dilate(imgThreshSmoothed, kernel, iterations=1)
    # Gaussian blur
    imgThreshSmoothed = cv2.GaussianBlur(imgThreshSmoothed, (5, 5), 0)
    #cv2.imshow('imgThreshSmoothed ', imgThreshSmoothed)
    # get Canny edges

    imgCanny = cv2.Canny(imgThreshSmoothed, 160, 80)
    #cv2.imshow('imgCanny ', imgCanny)
    if is_cv2():
        contours, hierarchy = cv2.findContours(imgCanny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    else:
        image, contours, hierarchy = cv2.findContours(imgCanny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    listOfContours = []
    if len(contours) != 0:
        for cnt in contours:
            # epsilon = 0.1 * cv2.arcLength(cnt, True)
            # print'epsilon',epsilon
            listOfContours.append(cv2.approxPolyDP(cnt, 6.7, True))

    listOfCones = []
    pose = Pose2D()
    poses = []
    loc = location_data()
    loc.distance_is_real = False
    dh, dw = h, w
    if depthImg is not None:
        loc.distance_is_real = True
        dh, dw = depthImg.shape
        #msg_str = "Image (%d, %d), Depth (%d, %d)" % (w, h, dw, dh);
        #rospy.loginfo(msg_str)

    for contour in listOfContours:
        hull = cv2.convexHull(contour)
        # print 'convexHull',len(temp)
        if (len(hull) >= 3 and convexHullIsPointingUp(hull)):
            listOfCones.append(hull)
            x, y, w, h = cv2.boundingRect(hull)
            pose.x = x + w/2 - image_centerX
            # Height is being measured top of screen to down so we need to invert y
            lowY = image_centerY - (y+h)
            pose.y = 256*lowY
            if depthImg is not None:
                # Depth image is 16UC1
                pose.y = depthImg[pose.x, lowY]
                rospy.logdebug('%d ==> %d' % (256*lowY, pose.y))

            # It should never happen that pose.y is 0 or negative
            if (pose.y > 0):
                pose.theta = round((pose.x * 1.0) / pose.y , 3)
                poses.append(pose)

    loc.poses = poses
    loc.header.stamp = rospy.Time.now()
    imghull = img.copy()
    cv2.drawContours(imghull, listOfCones, -1, (0, 255, 0), 3)
    if(len(listOfCones)):
        pub.publish(loc)

    return len(listOfCones), imghull

def find_in_images(loc='../images'):
    # get the files
    files = glob.glob(loc + '/*.jpg')

    rate = rospy.Rate(1) # One image per second
    for file in files:
        if args.debug:
            rospy.logdebug('Processing file %s' % file)
        count, imghull = find_cones(cv2.imread(file, -1))
        if args.debug:
            cv2.imshow('output', imghull)
            msg_str = 'Found %d Cones' % count
            rospy.logdebug(msg_str)
        rate.sleep()

def find_in_video(fileName):
    if(fileName == None):
      cap = cv2.VideoCapture(0)
    else:
      cap = cv2.VideoCapture(fileName)
      if(cap.isOpened() == False):
        rospy.logerr("Error: Could not open video file " + fileName + ". Using default device.")
        cap = cv2.VideoCapture(0)

    if(cap.isOpened() == False):
      rospy.logerr("Could not open default video device")
      return

    rate = rospy.Rate(10) # 10 frames per second
    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Display the resulting frame
        if(ret):
            count, imghull = find_cones(frame)
            if args.debug:
                cv2.imshow('output', imghull)
                msg_str = 'Found %d Cones' % count
                rospy.loginfo(msg_str)

        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
        rate.sleep()

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

class RosColorDepth:
    def __init__(self):
        self.node_name = "RosColorDepth"
        self.bridge = CvBridge()
        self.thread_lock = threading.Lock()
        colorCamInfo = message_filters.Subscriber("/camera/color/camera_info", CameraInfo)
        depthCamInfo = message_filters.Subscriber("/camera/depth/camera_info", CameraInfo)
        ts = message_filters.TimeSynchronizer([colorCamInfo, depthCamInfo], 10)
        ts.registerCallback(self.camInfoCallback)
        colorImage = message_filters.Subscriber("/camera/color/image_raw", Image)
        depthImage = message_filters.Subscriber("/camera/depth/image_raw", Image)
        ts = message_filters.TimeSynchronizer([colorImage, depthImage], 10)
        ts.registerCallback(self.imageCallback)
        rospy.loginfo("[%s] Initialized." %(self.node_name))
        rospy.spin()

    def camInfoCallback(self, colorCamInfo, depthCamInfo):
        self.colorCamInfo = colorCamInfo
        self.depthCamInfo = depthCamInfo

    def imageCallback(self, colorImage, depthImage):
        thread = threading.Thread(target=self.processImage, args=(colorImage, depthImage))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, colorImage, depthImage):
        if not self.thread_lock.acquire(False):
            return

        #print(colorImage.encoding, depthImage.encoding)
        cvRGB = self.bridge.imgmsg_to_cv2(colorImage, "bgr8")
        cvDepth = self.bridge.imgmsg_to_cv2(depthImage)

        dh, dw = cvDepth.shape[:2]
        ch, cw = cvRGB.shape[:2]
        if (ch != dh) and (cw != dw): 
            cvRGB = cv2.resize(cvRGB, (dw, dh))
            #cvDepth = cv2.resize(cvDepth, (cw, ch), interpolation = cv2.INTER_LINEAR)

        self.colorCamInfo.width = cw
        self.colorCamInfo.height = ch
        self.depthCamInfo.width = cw
        self.depthCamInfo.height = ch

        try:
            count, imghull = find_cones(cvRGB, cvDepth)
            ts = rospy.Time.now()
            self.colorCamInfo.header.stamp = ts
            self.depthCamInfo.header.stamp = ts
            colorCIPub.publish(self.colorCamInfo)
            depthCIPub.publish(self.depthCamInfo)
            colorMsg = self.bridge.cv2_to_imgmsg(imghull, "bgr8")
            colorMsg.header.stamp = ts
            colorMsg.header.frame_id = 'camera_link'
            colorPub.publish(colorMsg)
            depthMsg = depthImage
            depthMsg.header.stamp = ts
            depthMsg.header.frame_id = 'camera_link'
            depthPub.publish(depthMsg)
            if args.debug:
                #cv2.imshow('output', imghull)
                msg_str = 'Found %d Cones' % count
                rospy.loginfo(msg_str)
           
        except CvBridgeError as e:
            rospy.logerr(e)
            
        self.thread_lock.release()
    
def find_cones_main():
    #print(args.debug, args.image_dir, args.video_file)
    rospy.init_node('cone_finder')
    if args.use_ros_topic:
        r = RosColorDepth()
    else:
        if args.image_dir:
            args.debug = True
            find_in_images(args.image_dir)
            # No rospy.spin when working with image dir
        else:
            find_in_video(args.video_file)
            rospy.spin()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Find cones in video feed or images')
    parser.add_argument('--use_ros_topic', '-r', action='store_true', help='Use ROS topic')
    parser.add_argument('--image_dir', '-i', help='Find cones in images under specified directory')
    parser.add_argument('--debug', '-d', action='store_true', help='Show debug messages')
    parser.add_argument('video_file', nargs='?', help='Find cones in specified video file, use default video device if not specified')
    parser.parse_args(None, args)
    try:
      find_cones_main()

    except rospy.ROSInterruptException:
      pass
