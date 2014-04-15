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

def changeOri(x, y, z, w):
	right = baxter_interface.Limb('right')
	pose = right.endpoint_pose()
	curX = pose["orientation"].x
	curY = pose["orientation"].y
	curZ = pose["orientation"].z
	curW = pose["orientation"].w
	newX = curX + x
	newY = curY + y
	newZ = curZ + z
	newW = curW + w
	ori = Quaternion(newX, newY, newZ, newW)

	limb_joints = ik_solve("right", pose["location"], ori)

	if (limb_joints != -1):
		right.move_to_joint_positions(limb_joints)

def setOri(x, y, z, w):
	right = baxter_interface.Limb('right')

	limb_joints = ik_solve("right", right.endpoint_pose()["location"], Quaternion(x, y, z, w))

	if (limb_joints != -1):
		right.move_to_joint_positions(limb_joints)

def setWrist(theta):
	right = baxter_interface.Limb('right')
	rj = right.joint_names()
	wrist = rj[6]

	# Not sure if this is needed
	baxter_external_devices.getch()
	
	joint_command = {wrist: 0.0}

	for x in range(0, 300):
		right.set_joint_positions(joint_command)
	
