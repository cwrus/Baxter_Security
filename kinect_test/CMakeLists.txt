cmake_minimum_required(VERSION 2.8.3)
project(kinect_test)

set(CATKIN_DEPS roscpp rospy std_msgs message_runtime cv_bridge opencv2 openni_launch)
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
)

# Install the python scripts
catkin_python_setup()

# Add our message files
add_message_files(
  FILES
  Coords.msg
)

# Create the new messages
generate_messages()

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPS}
  DEPENDS ${SYSTEM_DEPS}
)

install(PROGRAMS src/find_red.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
