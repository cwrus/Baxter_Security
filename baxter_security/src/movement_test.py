#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import baxter_external_devices
import movement
from ik_solver import ik_solve
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

	
def main():
	rospy.init_node("baxter_movement_test", anonymous=True)

	right = baxter_interface.Limb('right')
	right.set_joint_position_speed(0.1)
	
	#movement.translateRel(0.104, 0)
	movement.moveRel(0, 0.05, 0)

if __name__ == '__main__':
	sys.exit(main())
