#!/usr/bin/env python
import sys
import rospy
import baxter_interface
import baxter_external_devices
import math
import movement

from baxter_security.msg import Coords

def setup():
	rospy.init_node("baxter_security_rhand_wrist_orient", anonymous=True)

	while not rospy.is_shutdown():
		message = rospy.wait_for_message("/lighter_coords", BaxterRHandCamCoords)
		coordCallBack(message)

def coordCallBack(data):
	theta = math.radians(data.theta)

	right = baxter_interface.Limb('right')
	right.set_joint_position_speed(1)
	rj = right.joint_names()
	wrist = rj[6]
	orientation = right.joint_angle(wrist)

	movement.setWrist(orientation + theta)

if __name__ == '__main__':
	setup()
