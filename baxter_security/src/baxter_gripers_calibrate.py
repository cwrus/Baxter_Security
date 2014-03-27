#!/usr/bin/env python

import rospy
import baxter_interface

def main():
	rs = baxter_interface.RobotEnable()
	init_state = rs.state().enabled
	left = baxter_interface.Gripper("left")
	right = baxter_interface.Gripper("right")

	left.reboot()
	right.reboot()
	left.calibrate()
	right.calibrate()

	rospy.init_node("baxter_security_calibrate_grippers")
if __name__ == '__main__'
	main()
