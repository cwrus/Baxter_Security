#!/usr/bin/env python
import sys
import rospy
import cv
import cv2
import threading
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from roslib import message
from kinect_test.msg import Coords, FloatCoords

class PosFind:
  def __init__(self):
    # Create this ros node
    rospy.init_node("baxter_security_find_xyz", anonymous=True)

    # Subscribe to the object coordinate data
    self.coords_sub = rospy.Subscriber("/lighter_coords", Coords, self.coordsCallback)

    # Subscribe to the point cloud data
    self.points_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pcCallback)

    # Create the publisher for the xyz
    self.pub = rospy.Publisher("kinect_lighter_coords", FloatCoords)
    self.msg = FloatCoords()

    # Spinlock to prevent concurrent writes to the pc data
    self.lock = threading.Lock()

    self.pc_data = 0

    rospy.spin()

  def pcCallback(self, data):
    # Engage the lock and then store the data
    self.lock.acquire()
    self.pc_data = data # get data from a genrator type

    # Release the lock
    self.lock.release()

  def coordsCallback(self, data):
    center = (data.x, data.y)

    # Lock the pc data while traversing it
    self.lock.acquire()
    if self.pc_data != 0:
      # find the distance to the POI
      xyz = pc2.read_points(self.pc_data, field_names=None, skip_nans=False, uvs=[[center[0],center[1]]])
      centerpoint = next(xyz)
      print centerpoint[0], centerpoint[1], centerpoint[2]
      #print(center, centerpoint)
      self.msg.x = centerpoint[0]
      self.msg.y = centerpoint[1]
      self.msg.z = centerpoint[2]
      self.pub.publish(self.msg)
    # Unlock the pc data when we are done
    self.lock.release()

if __name__ == '__main__':
    pf = PosFind()
