cmake_minimum_required(VERSION 3.0.2)
project(viactors_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  rospy
  std_msgs
  compliant_plugin
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
   LIBRARIES viactors_plugins compliant_plugin
   CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs message_runtime compliant_plugin
   DEPENDS roscpp gazebo_ros compliant_plugin
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
add_library(bavs_plugin 
  src/bavs_plugin.cpp )
target_link_libraries(bavs_plugin 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES} )

## Declare a C++ library
add_library(sea_plugin 
  src/sea_plugin.cpp )
target_link_libraries(sea_plugin 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES} )

## Declare a C++ library
add_library(qbmove_plugin 
  src/qbmove_plugin.cpp )
target_link_libraries(qbmove_plugin 
  ${catkin_LIBRARIES}  
  ${GAZEBO_LIBRARIES} )

## Declare a C++ library
add_library(awas_ii_plugin 
  src/awas_ii_plugin.cpp )
target_link_libraries(awas_ii_plugin 
  ${catkin_LIBRARIES}  
  ${GAZEBO_LIBRARIES} )

## Declare a C++ library
add_library(template_plugin 
  src/template_plugin.cpp )
target_link_libraries(template_plugin 
  ${catkin_LIBRARIES}  
  ${GAZEBO_LIBRARIES} )