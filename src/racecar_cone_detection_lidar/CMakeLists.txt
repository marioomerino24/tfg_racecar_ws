cmake_minimum_required(VERSION 3.0.2)
project(racecar_cone_detection_lidar)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  racecar_cone_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/cone_detector_node.py
    scripts/error_evaluator_node.py
    scripts/scan_to_cones.py
    scripts/cone_error_evaluator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
