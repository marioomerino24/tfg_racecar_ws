cmake_minimum_required(VERSION 3.0.2)
project(racecar_sim_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  gazebo_ros
  gazebo_msgs
)

catkin_package()

#############
## Install ##
#############

# Instala los scripts Python como ejecutables del paquete
catkin_install_python(PROGRAMS
  scripts/gazebo_odometry.py
  scripts/spawn_cones.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Instala los lanzadores
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Instala los mundos (SDF/world)
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
