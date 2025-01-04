cmake_minimum_required(VERSION 3.0.2)
project(humans)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  genmsg
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(DIRECTORY msg
  FILES
  Object.msg
  ObjectInteraction.msg
  Human.msg
  HumanInteraction.msg
  HumanWithInteractions.msg
  HumansWithInteractions.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/static_humans_scenario1.py src/static_humans_scenario2.py src/static_humans_scenario3.py src/static_humans_scenario4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
