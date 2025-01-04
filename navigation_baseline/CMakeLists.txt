cmake_minimum_required(VERSION 3.0.2)
project(navigation_baseline)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############
install(DIRECTORY
  config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
