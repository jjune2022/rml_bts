cmake_minimum_required(VERSION 3.0.2)
project(testdrive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
)


catkin_install_python(PROGRAMS
  src/testdrive.py
  src/testdrive2.py
  src/final_testdrive.py
  src/drive_gurobi.py
  src/stanley.py
  src/purepursuit.py
  src/purepursuit_v2.py
  src/lqr.py
  src/mpc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
