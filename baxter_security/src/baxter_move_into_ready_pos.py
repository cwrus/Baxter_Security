#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import baxter_external_devices
import movement
import math
from kinect_cam.msg import KinectFloatCoords
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class moveIntoReady:
  def __init__(self):
    # Create this ros node
    rospy.init_node("baxter_move_into_ready", anonymous=True)  

    # Subscribe to the kinect's coord publisher
    rospy.Subscriber("/kinect_lighter_coords", KinectFloatCoords, self.coordsCallback)

    rospy.spin()

  def coordsCallback(self, data):
    # Initilize the 4 pre-set starting areas
    bottomRight = [0.761010389869, -0.279305582713, 0.184718242161]
    topRight = [0.854727558874, -0.24278074904, 0.199642837114]
    bottomLeft = [0.760846322039, -0.133527073353, 0.1932059736]
    topLeft = [0.833713943767, -0.108011570937, 0.203745233174]
    shortest = [100, 100]
    # Find the closest approach point
    rightSide = 0.4
    leftSide = 0.04
    topEdge = 0.05
    bottomEdge = 0.31
    # Closer to right side
    if abs(data.x-rightSide) < abs(data.x-leftSide):
      if abs(data.y-topEdge) < abs(data.y-bottomEdge):
        shortest = topRight
      else:
        shortest = bottomRight
    #Closer to left side
    else:
      if abs(data.y-topEdge) < abs(data.y-bottomEdge):
        shortest = topLeft
      else:
        shortest = bottomLeft
    movement.moveTo(shortest[0], shortest[1], shortest[2])
    rospy.signal_shutdown("At approach posisition")

if __name__ == '__main__':
    rdy = moveIntoReady()
