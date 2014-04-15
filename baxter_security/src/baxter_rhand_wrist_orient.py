#!/usr/bin/env python
import sys
import rospy
import baxter_interface
import baxter_external_devices
import math
import movement

from baxter_security.msg import Coords

def setup():
	global moved
	moved = False
	rospy.init_node("baxter_security_rhand_wrist_orient", anonymous=True)

	global coords_sub
	coords_sub = rospy.Subscriber("/lighter_coords", Coords, coordCallBack)

	rospy.spin()

def coordCallBack(data):
	global moved

	if not moved:
		moved = True
		theta = math.radians(data.theta)

		right = baxter_interface.Limb('right')
		right.set_joint_position_speed(0.1)
		rj = right.joint_names()
		wrist = rj[6]
		orientation = right.joint_angle(wrist)
		error = theta - orientation

		print theta, orientation, error

		movement.setWrist(theta)

if __name__ == '__main__':
	setup()
