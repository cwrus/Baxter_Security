#!/usr/bin/env python

import rospy

import baxter_interface
import baxter_external_devices

def main():
	rospy.init_node("baxter_security_wrist_set_zero")
	rs = baxter_interface.RobotEnable()
	init_state = rs.state().enabled
	rs.enable()

	right = baxter_interface.Limb('right')
	rj = right.joint_names()
	wrist = rj[6]

	print("Current ori : %s" % right.joint_angle(wrist))

	right.set_joint_position_speed(0.3)

	for x in range(0, 300):
		baxter_external_devices.getch()
		joint_command = {wrist: 0.0}
		right.set_joint_positions(joint_command)

	print("cmd %s" % joint_command)

if __name__ == '__main__':
	main()
