#!/usr/bin/env python
#This file acts as a library for all of our arm movement commands. Each function within has
# a distinct purpose, and this is dependent on the iksolver.py file. 


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

global found = False
global delta = .01

# Move x,y,z meters relative to the current position
def moveRel(x, y, z):
	#get the current position and orientation of the arm
	right = baxter_interface.Limb('right')
	pose = right.endpoint_pose()
	curX = pose["position"].x
	curY = pose["position"].y
	curZ = pose["position"].z
	#we set the new x,y,z positions
	newX = curX + x
	newY = curY + y
	newZ = curZ + z
	#condense this position into a point variable
	loc = Point(newX, newY, newZ)
	#solve for the new joint positions given our current orientation and new x,y,z position
	limb_joints = ik_solve("right", loc, pose["orientation"]) 
	#if there's a valid solution, move to it. 
	if (limb_joints != -1):
		right.move_to_joint_positions(limb_joints)

# Moved to a specified XYZ location
def moveTo(x, y, z):
	right = baxter_interface.Limb('right')
	#solve for the joint positions given the desired x,y,z
	limb_joints = ik_solve("right", Point(x, y, z), right.endpoint_pose()["orientation"])
	#if there's a valid solution, move to it. 
	if (limb_joints != -1):
		right.move_to_joint_positions(limb_joints)

# Moved to a specified X location
def moveToX(x):
	right = baxter_interface.Limb('right')
	pose = right.endpoint_pose()
	curY = pose["position"].y
	curZ = pose["position"].z

	moveRel(x, curY, curZ)

# Moved to a specified Y location
def moveToY(y):
	right = baxter_interface.Limb('right')
	pose = right.endpoint_pose()
	curX = pose["position"].x
	curZ = pose["position"].z

	moveRel(curX, y, curZ)

# Moved to a specified Z location
def moveToZ(z):
	right = baxter_interface.Limb('right')
	pose = right.endpoint_pose()
	curX = pose["position"].x
	curY = pose["position"].y

	moveRel(curX, curY, z)

# Translate the hand around on the XY plane relative to the current position
def translateRel(x, y):
	#maintain height, and move x and y meters relative to currnt position
	moveRel(x, y, 0)

# Translate the hand around on the XY plane to a specific location
def translateTo(x, y):
	right = baxter_interface.Limb('right')
	#get the current position, and maintain the z value
	pose = right.endpoint_pose()
	curZ = pose["position"].z
	#move to the absolute position of x,y maintaining height and orientation
	moveTo(x, y, curZ)

# Set the wrist to the specified theta value
def setWrist(theta):
	#get variables for the wrist and right arm. 
	right = baxter_interface.Limb('right')
	rj = right.joint_names()
	wrist = rj[6]

	joint_command = {}
	#form a joint command consisting of the current thetas of the robot (no movement)
	for i in range(1, len(rj)):
		joint_command[rj[i]] = right.joint_angle(rj[i])
	#Then change the theta of the wrist to the desired value
	joint_command[wrist] = theta
	#move to this joint command, maintaining all thetas but changing the wrist. 
	right.move_to_joint_positions(joint_command)
	
def getClose(x, y, z):
	global found, delta
	right = baxter_interface.Limb('right')
	pose = right.endpoint_pose()
	curX = pose["position"].x
	curY = pose["position"].y
	curZ = pose["position"].z

	if (not closeTo(x,y,z) and (not found):
		newX = curX + x
		newY = curY + y
		newZ = curZ + z
		#condense this position into a point variable
		loc = Point(newX, newY, newZ)
		#solve for the new joint positions given our current orientation and new x,y,z position
		limb_joints = ik_solve("right", loc, pose["orientation"]) 
		#if there's a valid solution, move to it. 
		if (limb_joints != -1):
			found = True
			right.move_to_joint_positions(limb_joints)
		else:
			getClose(x+delta,y,z)
			getClose(x,y+delta,z)
			getClose(x,y,z+delta)


def closeTo(x, y, z):
	err = .1
	right = baxter_interface.Limb('right')
	pose = right.endpoint_pose()
	curX = pose["position"].x
	curY = pose["position"].y
	curZ = pose["position"].z
	if (curX < x+err and curX > x-err and curY < y+err and curY > y-err and curZ < z+err and curZ > z-err):
		return True
	else:
		return False
