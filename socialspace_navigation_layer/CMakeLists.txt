cmake_minimum_required(VERSION 3.0.2)
project(socialspace_navigation_layer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  costmap_2d
  dynamic_reconfigure
  std_msgs
  geometry_msgs
  humans
  genmsg
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(DIRECTORY srv
  FILES
  SocialspaceFunction.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  humans
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options(
  cfg/SocialspaceLayer.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
  INCLUDE_DIRS include
  LIBRARIES socialspace_navigation_layer
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(socialspace_layer src/socialspace_layer.cpp)
target_link_libraries(socialspace_layer ${catkin_LIBRARIES})
add_dependencies(socialspace_layer socialspace_navigation_layer_generate_messages_cpp socialspace_navigation_layer_gencfg)

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  src/socialspace_function.py src/lib_socialspace_function.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  socialspace_layer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY
  include/socialspace_navigation_layer/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
