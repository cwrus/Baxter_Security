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

# Move relative to the current position
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

# Moved to a specified XYZ location
def moveTo(x, y, z):
	right = baxter_interface.Limb('right')
	
	limb_joints = ik_solve("right", Point(x, y, z), right.endpoint_pose()["orientation"])

	if (limb_joints != -1):
		right.move_to_joint_positions(limb_joints)

# Translate the hand around on the XY plane relative to the current position
def translateRel(x, y):
	moveRel(x, y, 0)

# Translate the hand around on the XY plane to a specific location
def translateTo(x, y):
	right = baxter_interface.Limb('right')
	pose = right.endpoint_pose()
	curZ = pose["position"].z

	moveTo(x, y, curZ)

# Set the wrist to the specified theta value
def setWrist(theta):
	right = baxter_interface.Limb('right')
	rj = right.joint_names()
	wrist = rj[6]

	joint_command = {}
	for i in range(1, len(rj)):
		joint_command[rj[i]] = right.joint_angle(rj[i])
	joint_command[wrist] = theta

	right.move_to_joint_positions(joint_command)
	
