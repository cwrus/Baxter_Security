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
from kinect_test.msg import Coords

class PosFind:
  def __init__(self):
    # Create this ros node
    rospy.init_node("baxter_security_find_xyz", anonymous=True)

    # Subscribe to the red coordinate data
    self.coords_sub = rospy.Subscriber("/baxter_security/lighter_coords", Coords, self.coordsCallback)

    # Subscribe to the point cloud data
    self.points_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pcCallback)

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

    # find the distance to the POI
    xyz = pc2.read_points(self.pc_data, field_names=None, skip_nans=False, uvs=[[center[0],center[1]]])
    centerpoint = next(xyz)

    print(center, centerpoint)

    # Unlock the pc data when we are done
    self.lock.release()

if __name__ == '__main__':
    pf = PosFind()