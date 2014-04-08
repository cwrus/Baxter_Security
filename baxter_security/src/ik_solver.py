#!/usr/bin/env python

import sys
import rospy
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

def ik_solve(limb, pos, orient):
	# Setup the IKSolver service
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	poses = {
		str(limb): PoseStamped(header=hdr,
			pose=Pose(position=pos, orientation=orient))
	}

	ikreq.pose_stamp.append(poses[limb])
	try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 1
	if (resp.isValid[0]):
		#print("SUCCESS - Valid Joint Solution Found:")
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		#print limb_joints
		return limb_joints
	else:
		print ("INVALID POSE - No Valid Joint Solution Found.")

	return -1
