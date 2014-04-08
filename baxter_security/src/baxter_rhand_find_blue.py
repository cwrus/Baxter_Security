#!/usr/bin/env python

import sys
import rospy
import roslib
import cv
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from roslib import message
from kinect_test.msg import Coords

class RedDetect:
  def __init__(self):
    # Create this ros node
    rospy.init_node("baxter_security_hand_find_red", anonymous=True)
  
    # Create the publisher
    self.pub = rospy.Publisher("lighter_coords", Coords)
    self.msg = Coords()

    # Setup the OpenCV <--> ROS bridge
    self.bridge = CvBridge()

    # Subcribe to baxter's right hand image node
    rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.callback)

    rospy.spin()

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    cv_image=np.array(cv_image, dtype=np.uint8) #convert image to a numpy
    # Begin getting color
    cv_image = cv2.GaussianBlur(cv_image, (3,3), 0) # Smooth the picture
    cv_imageHSV = np.zeros((cv_image.shape),np.uint8) # Creating a blank image of cv_image size
    cv_imageHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # Change color format
    cv_imageThresh = np.zeros((cv_image.shape), np.uint8)
    cv_imageThresh = cv2.inRange(cv_imageHSV, np.array((110, 120, 200)), np.array((120,256,256))) # Convert the HSV image to binary, getting red
    cv_imageThresh = cv2.GaussianBlur(cv_imageThresh, (3,3), 0) # Smooth the output picture
    # End getting color

    cv2.namedWindow("Red")
    cv2.imshow("Red", cv_imageThresh)
    cv2.namedWindow("Video")
    cv2.imshow("Video", cv_image)
    cv2.waitKey(3)

    # Fitting rectangle
    contour, hierarchy = cv2.findContours(cv_imageThresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contour:
      approx = cv2.approxPolyDP(cnt, cv2.arcLength(cnt, True)*0.02, True)
      if len(approx) == 4 and cv2.contourArea(cnt) > 50: #need to tune this
        cv2.drawContours(cv_image, [cnt], 0, 255, 2)
        moments = cv2.moments(cnt, True)
        if(len(filter(lambda x: x==0, moments.values())) > 0):
          continue
        center = (moments['m10']/moments['m00'], moments['m01']/moments['m00'])
        center = map(lambda x: int(round(x)), center)
    
        cv2.namedWindow("Shape")
        cv2.imshow("Shape", cv_image)
        cv2.waitKey(3)
        self.msg.x = center[0]
        self.msg.y = center[1]
        self.pub.publish(self.msg)

if __name__ == '__main__':
  rd = RedDetect()
