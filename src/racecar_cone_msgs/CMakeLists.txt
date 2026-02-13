cmake_minimum_required(VERSION 3.0.2)
project(racecar_cone_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(FILES
  Cone.msg
  ConeArray.msg
  ConePair.msg
  ConePairArray.msg
  MidpointArray.msg
  Centerline.msg
  TrackMetrics.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
