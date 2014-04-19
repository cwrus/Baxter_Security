#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import baxter_external_devices
import movement
import math
from kinect_test.msg import KinectFloatCoords
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
    bottomRight = [0.716638893979, -0.323182332641]
    topRight = [0.826942198613, -0.337988538941]
    bottomLeft = [0.756998869647, -0.163337925954]
    topLeft = [0.839202464355, -0.163838047031]
    # Predetermined offsets from the kinect frame
    xoff = 0.9
    yoff = 0
    zoff = 0.7124
    shortest = [100, 100]
    upperLim = 0.354
    middle = 0.232
    # Find the closest approach point
    rightSide = 0.4
    leftSide = 0.04
    topEdge = 0.05
    bottomEdge = 0.31
    # Closer to right side
    print data.x, data.y
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
	
    print shortest 
    # Get the current position
    right = baxter_interface.Limb('right')
    pose = right.endpoint_pose()
    curX = pose["position"].x
    curY = pose["position"].y
    curZ = pose["position"].z
    movement.translateTo(shortest[0], shortest[1])
    rospy.signal_shutdown("At approach posisition")

if __name__ == '__main__':
    rdy = moveIntoReady()
