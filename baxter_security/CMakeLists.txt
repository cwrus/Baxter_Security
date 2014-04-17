cmake_minimum_required(VERSION 2.8.3)
project(baxter_security)

set(CATKIN_DEPS roscpp rospy std_msgs message_runtime cv_bridge opencv2 openni_launch baxter_interface)
set(SYSTEM_DEPS )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  openni_launch
  roscpp
  rospy
  std_msgs
  message_generation
  baxter_interface
)


# Install the python scripts
catkin_python_setup()

# Add our message files
add_message_files(
  FILES
  BaxterRHandCamCoords.msg
)

# Create the new messages
generate_messages()

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPS}
  DEPENDS ${SYSTEM_DEPS}
)

install(PROGRAMS src/baxter_gripers_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
