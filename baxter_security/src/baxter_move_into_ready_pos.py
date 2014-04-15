#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import baxter_external_devices
import movement
from kinect_test.msg import Coords
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
    rospy.Subscriber("/baxter_security/kinect_lighter_coords", Coords, self.coordsCallback)

    rospy.spin()

  def coordsCallback(self, data):
    # Predetermined offsets from the kinect frame
    xoff = 0.899
    yoff = 0
    zoff = 0.7124
 
    # Get the current position
    right = baxter_interface.Limb('right')
    pose = right.endpoint_pose()
    curX = pose["position"].x
    curY = pose["position"].y
    curZ = pose["position"].z

    xready = xoff + data.x 
    yready = yoff + data.y
    zready = data.z - zoff

    # Move to the desired position, xy, then z
    #right = baxter_interface.Limb('right')
    right.set_joint_position_speed(0.05)
    movement.moveTo(xready, yready, curZ)
    movement.moveTo(xready, yready, zready)

if __name__ == '__main__':
    rdy = moveIntoReady()
