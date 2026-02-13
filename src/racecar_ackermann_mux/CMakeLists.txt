cmake_minimum_required(VERSION 3.0.2)
project(racecar_ackermann_mux)

find_package(catkin REQUIRED COMPONENTS rospy ackermann_msgs)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/ackermann_cmd_simple_mux.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
