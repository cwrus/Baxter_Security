#!/usr/bin/env python
import sys
import rospy
import baxter_interface
import baxter_external_devices
import math
import movement
from kinect_test.msg import KinectFloatCoords
from baxter_security.msg import BaxterRHandCamCoords
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

global wristDone
global armDone
global graspDone
global kinectZ

def setup():
	global wristDone
        global armDone 
	global graspDone
        wristDone = False	
        armDone = False
	graspDone = False

	rospy.init_node("baxter_security_rhand_orient", anonymous=True)

  	rospy.Subscriber("/kinect_lighter_coords", KinectFloatCoords, setZcallback)


	while not rospy.is_shutdown() and not graspDone:
		message = rospy.wait_for_message("/lighter_coords", BaxterRHandCamCoords)
		coordCallBack(message)

def setZcallback(data):
	global kinectZ
	kinectZ = data.z	

def coordCallBack(data):
        global wristDone
        global armDone
	global graspDone

	heightScale = 0.00075
	widthScale = 0.00075
	tol = 20
	wristTol = 0.02
	x = data.x
	y = data.y
	height = data.height
	width = data.width
	heightError = height/2 - y - 50
	widthError = width/2 - x - 30
	theta = math.radians(data.theta)

	right = baxter_interface.Limb('right')
	right.set_joint_position_speed(.2)
	rj = right.joint_names()
	wrist = rj[6]
	orientation = right.joint_angle(wrist)

	#if True:
		#print right.endpoint_pose()
	if not armDone:
		if (abs(widthError) > tol and abs(heightError) > tol):
			movement.translateRel(heightError*heightScale, widthError*widthScale)
		elif abs(widthError) > tol:
			movement.translateRel(0, widthError*widthScale)
		elif abs(heightError) > tol:
			movement.translateRel(heightError*heightScale, 0)
		else:
			print "Arm is good"
       			armDone = True 
	
	if not wristDone:
		if (abs(theta) > wristTol):
			#if (orientation + theta < -math.pi):
				#theta = theta + math.pi
			#elif (orientation + theta > math.pi):
				#theta = theta - math.pi
			print orientation, orientation + theta
			movement.setWrist(orientation + theta)
		else:
			print "Wrist is good"
        		wristDone = True

        if wristDone == True and armDone == True and not graspDone:
		right = baxter_interface.Limb('right')
    		pose = right.endpoint_pose()
		curZ = pose["position"].z
		objHeightOffTable = 0.9144 - kinectZ           
		targetZ = objHeightOffTable - 0.202
		targetZ = targetZ + 0.065 #Slight tolerance
		print "Down"
                movement.moveRel(0,0,targetZ-curZ)
		print "Up"
		baxter_interface.Gripper("right").close(True)
		movement.moveToZ(0.15)
		graspDone = True

if __name__ == '__main__':
	setup()
