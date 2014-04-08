#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import baxter_external_devices
from ik_solver import ik_solve
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

def moveRel(x, y, z):
	right = baxter_interface.Limb('right')
	pose = right.endpoint_pose()
	curX = pose["position"].x
	curY = pose["position"].y
	curZ = pose["position"].z
	newX = curX + x
	newY = curY + y
	newZ = curZ + z
	loc = Point(newX, newY, newZ)

	limb_joints = ik_solve("right", loc, pose["orientation"]) 

	if (limb_joints != -1):
		right.move_to_joint_positions(limb_joints)

def moveTo(x, y, z):
	right = baxter_interface.Limb('right')
	
	limb_joints = ik_solve("right", Point(x, y, z), right.endpoint_pose()["orientation"])

	if (limb_joints != -1):
		right.move_to_joint_positions(limb_joints) 

def main():
	# Init this ros node and enable Baxter
	rospy.init_node('move_right_hand')
	rs = baxter_interface.RobotEnable()
	rs.enable()

	# Get Baxter's right arm and set a max speed
	right = baxter_interface.Limb('right')
	rightGripper = baxter_interface.Gripper('right')
	right.set_joint_position_speed(1.0)

	pose = right.endpoint_pose()
	print "Current pos: ", pose
	#moveRel1(0, 0, -.1)
	#moveTo(.6, -.2, .2)

if __name__ == '__main__':
    sys.exit(main())
