cmake_minimum_required(VERSION 3.0.2)
project(path)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  nav_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
)

catkin_package(
)


catkin_install_python(PROGRAMS
  src/path_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
