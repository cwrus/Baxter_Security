from subprocess import call

# Calibrate the grippers
call(["rosrun","bexter_security","baxter_grippers_calibrate"])
# Start the kinect/openni 
call(["roslaunch","openni_launch","openni.launch"])
# Use the kinect to find blue lighter
call(["rosrun","kinect_cam","find_blue.py"])
# Use the kinect to find the xyz center of the lighter
call(["rosrun","kinect_cam","find_xyz.py"])
# Move Baxter's right hand into a position where it can see the lighter
call(["rosrun","baxter_security","baxter_move_into_ready_pos.py"])
# Start Baxter's hand camera and locate the lighter
call(["rosrun","baxter_security","baxter_rhand_find_blue.py"])
# Refine Baxter's location and grab the lighter, pulling it out of the luggage
call(["rosrun","baxter_security","baxter_rhand_orient.py"])




