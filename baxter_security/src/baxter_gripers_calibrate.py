#!/usr/bin/env python
#this code is used to calibrate the grippers, and should be run before any actuation is done. 
import rospy
import baxter_interface

def main():
	#initiate the node for the gripper calibration
	rospy.init_node("baxter_security_calibrate_grippers")
	
	#the following 2 commands enable baxter. 
	rs = baxter_interface.RobotEnable()
	init_state = rs.state().enabled
	#we then set variables for his left and right gripper
	left = baxter_interface.Gripper("left")
	right = baxter_interface.Gripper("right")
	#we reboot and call the automated calibrate function
	left.reboot()
	right.reboot()
	left.calibrate()
	right.calibrate()

if __name__ == '__main__':
	main()
