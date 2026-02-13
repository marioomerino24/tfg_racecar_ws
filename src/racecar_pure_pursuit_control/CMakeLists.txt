cmake_minimum_required(VERSION 3.0.2)
project(racecar_pure_pursuit_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/pure_pursuit_controllers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
