#!/usr/bin/env python
import sys
import rospy
import baxter_interface
import baxter_external_devices
import math
import movement

from baxter_security.msg import Coords

def setup():
	rospy.init_node("baxter_security_rhand_orient", anonymous=True)

	while not rospy.is_shutdown():
		message = rospy.wait_for_message("/lighter_coords", Coords)
		coordCallBack(message)

def coordCallBack(data):
	heightScale = 0.00075
	widthScale = 0.00075
	tol = 20
	wristTol = 0.02
	x = data.x
	y = data.y
	height = data.height
	width = data.width
	heightError = height/2 - y - 50
	widthError = width/2 - x - 30
	theta = math.radians(data.theta)

	right = baxter_interface.Limb('right')
	right.set_joint_position_speed(.2)
	rj = right.joint_names()
	wrist = rj[6]
	orientation = right.joint_angle(wrist)

	#if True:
		#print right.endpoint_pose()
	if (abs(widthError) > tol and abs(heightError) > tol):
		movement.translateRel(heightError*heightScale, widthError*widthScale)
	elif abs(widthError) > tol:
		movement.translateRel(0, widthError*widthScale)
	elif abs(heightError) > tol:
		movement.translateRel(heightError*heightScale, 0)
	else:
		print "Arm is good"

	if (abs(theta) > wristTol):
		movement.setWrist(orientation + theta)
	else:
		print "Wrist is good"

if __name__ == '__main__':
	setup()
