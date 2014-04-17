#!/usr/bin/env python

import rospy
import baxter_interface
import time
from std_msgs.msg import Float32

# Yellow: Red 100, Green 65

def main():
	rospy.init_node("baxter_security_christmas")

	red = rospy.Publisher("/robot/sonar/head_sonar/lights/set_red_level", Float32)
	green = rospy.Publisher("/robot/sonar/head_sonar/lights/set_green_level", Float32)

	redMsg = Float32()
	greenMsg = Float32()

	while not rospy.is_shutdown():
		for i in range(0, 100):
			green.publish(i)
			time.sleep(0.025)
		for i in range(0, 100):
			green.publish(100 - i)
			time.sleep(0.025)

if __name__ == '__main__':
	main()
