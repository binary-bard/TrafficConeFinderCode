#
# Common functions for cone detection and seeking
#
import numpy as np
import cv2, time, rospy

# Needed for publishing the messages
from cone_finder.msg import pose_data
from cone_finder.msg import location_msgs as location_data
 
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
       
class ConeFinder:
    codec = 'XVID'

    def __init__(self, min_area=100):
        self.firstTime = True
        self.rgbOut = None
        self.depthOut = None
        self.min_area = min_area
    
    def _initCapture(self, frame, outFile):
        (h, w) = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        capOut = cv2.VideoWriter(outFile, fourcc, 15.0, (w, h), True)
        if(capOut.isOpened() == False):
            capOut = None
            rospy.loginfo("Could not open %s file to write video" % outFile)    
        
        return capOut            

    #Returns depth range tuple (min, max)
    def _getHullDepth(self, hull, depthImg=None):
        if(depthImg is None):
            return (0, 0)

        h = depthImg.shape[:1]
        depthList = []
        # Get bounding box of the hull
        for point in hull:
            depth = depthImg[point.x, point.y]
            # R200 provides depth from 50cm to 3.5m typically and each unit is mm
            if(depth > 0):
                depthList.append(depth)

        # If we have most of the points with depth, we will assume the rest were error
        if(len(depthList) > len(hull)/2):
            depthList = sorted(depthList)
            return (depthList[0], depthList[-1])

        return (0, 0)
        
    def _convexHullIsPointingUp(self, hull):
        (centerX, centerY), (w, h), angle = cv2.minAreaRect(hull)
        # Remove any cones with size less than certain threshold
        if(h*w < self.min_area):
            return False
        
        # Our cones are tall, rather than high
        aspectRatio = float(w) / h
        if aspectRatio > 0.9:
            return False

        # Very inclined cone, drop them
        if(angle > 30 or angle < -30):
            return False
        
        listOfPointsAboveCenter = []
        listOfPointsBelowCenter = []

        # step through all points in convex hull
        for point in hull:
            # and add each point to
            # list of points above or below vertical center as applicable
            if point[0][1] < centerY:
                listOfPointsAboveCenter.append(point)

            if point[0][1] >= centerY:
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

    def _process_orange_color(self, img):
        # convert to HSV color space, this will produce better color filtering
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Threshold on low range of HSV red
        low_redl = np.array([0, 135, 90])
        low_redh = np.array([19, 255, 255])
        imgThreshLow = cv2.inRange(imgHSV, low_redl, low_redh)

        # threshold on high range of HSV red
        high_redl = np.array([160, 135, 90])
        high_redh = np.array([179, 255, 255])
        imgThreshHigh = cv2.inRange(imgHSV, high_redl, high_redh)

        # combine low range red thresh and high range red thresh
        return cv2.bitwise_or(imgThreshLow, imgThreshHigh)

    def captureFrames(self, cvRGB, cvDepth):
        if(self.firstTime):    
            # Initialize capture devices
            timestr = time.strftime("%Y%m%d-%H%M%S")
            rgbOutFile = 'rgb%s.avi' % timestr
            depthOutFile = 'depth%s.avi' % timestr
            self.rgbOut = self._initCapture(cvRGB, rgbOutFile)
            self.depthOut = self._initCapture(cvDepth, depthOutFile)
            self.firstTime = False

        if(self.rgbOut is not None and self.rgbOut.isOpened() and cvRGB is not None):
            self.rgbOut.write(cvRGB)
        if(self.depthOut is not None and self.depthOut.isOpened() and cvDepth is not None):
            self.depthOut.write(cv2.cvtColor((cvDepth/256).astype('uint8'), cv2.COLOR_GRAY2BGR))

    def find_cones(self, img, depthImg=None):
        h, w = img.shape[:2]
        
        image_centerX = w/2
        image_centerY = h  # y goes down from top
            
        # Process orange color and convert to gray image
        imgThresh = self._process_orange_color(img)
                
        if is_cv2():
            contours, hierarchy = cv2.findContours(imgThresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        else:
            image, contours, hierarchy = cv2.findContours(imgThresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        listOfHullsAndArea = []
        if len(contours) != 0:
            for cnt in contours:
                epsilon = 0.1 * cv2.arcLength(cnt, True)
                # print'epsilon',epsilon
                contour = cv2.approxPolyDP(cnt, epsilon, True)
                # Find convex hulls.
                hull = cv2.convexHull(contour, returnPoints=True)
                # See how the hull looks as a triangle
                # tri = cv2.minEnclosingTriangle(hull)
                # get the depth for the hull. Is it one value or multiple?
                depthRange = self._getHullDepth(hull)
                # We need to sort and store the contours by proximity of their centroids
                listOfHullsAndArea.append((hull, cv2.contourArea(hull), depthRange))

        listOfCones = []
        poses = []

        # Sort the list by decreasing area
        listOfHullsAndArea = sorted(listOfHullsAndArea, key=lambda pair: pair[1], reverse=True)
        for (hull, area, (dMin, dMax)) in listOfHullsAndArea:
            # print 'convexHull',len(temp)
            if (len(hull) >= 3 and self._convexHullIsPointingUp(hull)):
                listOfCones.append(hull)
                x, y, w, h = cv2.boundingRect(hull)
                pose = pose_data()
                pose.x = x + w/2 - image_centerX
                pose.w = w
                # Height is being measured top of screen to down so we need to invert y
                pose.y = (image_centerY - (y+h))
                pose.h = h
                pose.z = dMin   # But this is the hypotenuse
                pose.d = dMax - dMin
                pose.area = area
                poses.append(pose)

        return (poses, listOfCones)

# We will get angle between +pi/2 to -pi/2 for steering
# We will get 480 pixels range for throttle but should limit this
class ConeSeeker:
    # Typically less than 1 unless the range isn't responsive
    conf_decay_factor = 0.80
    prev_pos_confs = []

    def __init__(self):
        self.prev_pos_confs = []
    
    def _update_prev_poses(self):
        new_pos_confs = []
        for (prev_pose, confidence, frame) in self.prev_pos_confs:
            confidence *= self.conf_decay_factor
            frame += 1
            new_pos_confs.append((prev_pose, confidence, frame))

        new_pos_confs = sorted(new_pos_confs, key=lambda pose: pose[1], reverse=True)
        # Keep only top 16 items
        self.prev_pos_confs = new_pos_confs[0:16]
        
    def _getConfFromOldFrames(self, pose):
        x1 = pose.x - pose.w/2
        x2 = pose.x + pose.w/2
        y1 = pose.y
        y2 = pose.y + pose.h
        conf = 0.0
        matched_poses = []
        for (id, (prev_pose, prev_conf, frame)) in enumerate(self.prev_pos_confs):
            if(frame == 0):
                continue
            old_x1 = prev_pose.x - prev_pose.w/2
            old_x2 = prev_pose.x + prev_pose.w/2
            old_y1 = prev_pose.y
            old_y2 = prev_pose.y + prev_pose.h
            dx = min(x2, old_x2) - max(x1, old_x1)
            dy = min(y2, old_y2) - max(y1, old_y1)
            if (dx>=0) and (dy>=0):
                conf += prev_conf * (dx*dy*1.0)/(prev_pose.w*prev_pose.h)
                matched_poses.append(id)
              
        return (conf, matched_poses)
          
    def seek_cone(self, poses):
        # Compute confidence for each hull by area and h distance
        maxArea = max(pose.area for pose in poses)
        self._update_prev_poses()
        
        all_matches = []
        new_pos_confs = []
        for pose in poses:
          oldConf, matched_poses = self._getConfFromOldFrames(pose)
          all_matches.extend(matched_poses)
          # Need to figure out appropriate weightage for area and distance
          # Scale distance as farther objects will use less pixels
          pd = 1 + (pose.x/80.0)**2 + (pose.y/120.0)**2
          # Find this cone among cones from previous frames and use the confidence
          conf = 1/pd + pose.area/(4.0*maxArea) + oldConf
          new_pos_confs.append((pose, conf, 0))
          #print('x=%d, y=%d, pd=%d, ar=%f, cf=%f, ocf=%f' % (pose.x, pose.y, pd, (pose.area*1.0/maxArea), conf, oldConf))
                
        all_matches = list(set(all_matches))
        for id in sorted(all_matches, reverse=True):
          self.prev_pos_confs.pop(id)

        # Sort the new list by confidence and descending
        self.prev_pos_confs.extend(new_pos_confs)
        self.prev_pos_confs = sorted(self.prev_pos_confs, key=lambda pose: pose[1], reverse=True)
        return self.prev_pos_confs[0]


