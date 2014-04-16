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
	x = data.x
	y = data.y
	height = data.height
	width = data.width
	heightScale = 0.001
	widthScale = 0.001
	heightError = height/2 - y
	widthError = width/2 - x
	theta = math.radians(data.theta)

	right = baxter_interface.Limb('right')
	right.set_joint_position_speed(1)
	rj = right.joint_names()
	wrist = rj[6]
	orientation = right.joint_angle(wrist)

	#movement.setWrist(orientation + theta)

	#movement.translateRel(widthError*widthScale, 0)
	print widthError*widthScale

if __name__ == '__main__':
	setup()
