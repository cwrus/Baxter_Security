#!/usr/bin/env python
#This is essentially a library that is used to calculate an IK solution for a given endpoint. 
#For examples on how this is used, see movement.py
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
#This function is called given a limb, (left/right) a desired position, and a desiired orientation.
def ik_solve(limb, pos, orient):
	# Setup the IKSolver service
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	# Create the message to be sent to the solver
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	poses = {
		str(limb): PoseStamped(header=hdr,
			pose=Pose(position=pos, orientation=orient))
	}
	ikreq.pose_stamp.append(poses[limb])
	
	# Try to send the message to the iksolver and hope it returns a valid solution
	try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 1

	# If there is a valid joint pose then return it
	if (resp.isValid[0]):
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		return limb_joints
	# Otherwise let the operators know
	else:
		print ("INVALID POSE - No Valid Joint Solution Found.")

	return -1
