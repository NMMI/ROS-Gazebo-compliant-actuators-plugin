cmake_minimum_required(VERSION 2.8.3)
project(plugin_manager)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


##########################
## Declare ROS messages ##
##########################

add_message_files(
  FILES
  QbAdvanceRefEq.msg
  state_info.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

##############################
## Declare a C++ executable ##
##############################
# 
add_executable(plugin_manager_node src/plugin_manager.cpp)
add_dependencies(plugin_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plugin_manager_node ${catkin_LIBRARIES})
