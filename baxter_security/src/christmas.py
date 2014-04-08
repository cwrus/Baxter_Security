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
		red.publish(100.0)
		green.publish(0.0)

		time.sleep(0.1)

		red.publish(0.0)
		green.publish(100.0)

		time.sleep(0.1)

if __name__ == '__main__':
	main()
