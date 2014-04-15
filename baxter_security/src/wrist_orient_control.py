#!/usr/bin/env python

import rospy

import baxter_interface
import baxter_external_devices
import movement

def main():
	rospy.init_node("baxter_security_wrist_set_zero")
	rs = baxter_interface.RobotEnable()
	init_state = rs.state().enabled
	rs.enable()

	right = baxter_interface.Limb('right')
	right.set_joint_position_speed(0.3)
	rj = right.joint_names()
	wrist = rj[6]

	joint_command = {}
	for i in range(1, len(rj)):
		joint_command[rj[i]] = right.joint_angle(rj[i])
	joint_command[wrist] = 0.0
	right.move_to_joint_positions(joint_command)

if __name__ == '__main__':
	main()
