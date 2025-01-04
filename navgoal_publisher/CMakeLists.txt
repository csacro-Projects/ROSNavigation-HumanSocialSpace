cmake_minimum_required(VERSION 3.0.2)
project(navgoal_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  src/navgoal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
