cmake_minimum_required(VERSION 3.0.2)
project(racecar_track_geometry)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs visualization_msgs nav_msgs tf racecar_cone_msgs
)

catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs visualization_msgs nav_msgs tf racecar_cone_msgs)
include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/midpoints_node.py
  scripts/centerline_fit_node.py
  scripts/centerline_to_path_node.py
  scripts/bins_live_view.py
  scripts/track_viz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
