cmake_minimum_required(VERSION 3.0.2)
project(racecar_pure_pursuit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  visualization_msgs
  std_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/pure_pursuit_target_selector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
