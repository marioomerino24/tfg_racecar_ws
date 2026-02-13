cmake_minimum_required(VERSION 3.0.2)
project(racecar_sim_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  controller_manager
  joint_state_controller
  effort_controllers
  position_controllers
  velocity_controllers
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/keyboard_teleop.py
    scripts/servo_commands.py
    scripts/keyboard_teleop2.py
    scripts/teleop_debug.py
    scripts/teleop_fix_path.py
    scripts/teleop_debug_fix.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
