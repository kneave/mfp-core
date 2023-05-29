cmake_minimum_required(VERSION 3.0.2)
project(mfp-core)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  ros_redboard
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  ros_redboard
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs ros_redboard
  DEPENDS ros_redboard
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

