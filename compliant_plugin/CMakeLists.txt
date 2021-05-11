cmake_minimum_required(VERSION 3.0.2)
project(compliant_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  rospy
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)

add_message_files(
    FILES
  state_info.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES compliant_plugin
   CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs message_runtime
   DEPENDS roscpp gazebo_ros 
#  DEPENDS system_lib  
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/compliant_plugin.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} 
   ${GAZEBO_LIBRARIES}
 )


