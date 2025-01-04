cmake_minimum_required(VERSION 3.0.2)
project(people_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  people_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  src/static_people_scenario1.py src/static_people_scenario2.py src/static_people_scenario3.py src/static_people_scenario4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
