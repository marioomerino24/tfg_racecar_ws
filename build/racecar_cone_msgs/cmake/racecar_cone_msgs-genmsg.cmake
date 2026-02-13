# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "racecar_cone_msgs: 7 messages, 0 services")

set(MSG_I_FLAGS "-Iracecar_cone_msgs:/ws/src/racecar_cone_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(racecar_cone_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePair.msg" NAME_WE)
add_custom_target(_racecar_cone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "racecar_cone_msgs" "/ws/src/racecar_cone_msgs/msg/ConePair.msg" "geometry_msgs/Point:racecar_cone_msgs/Cone:std_msgs/Header"
)

get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Centerline.msg" NAME_WE)
add_custom_target(_racecar_cone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "racecar_cone_msgs" "/ws/src/racecar_cone_msgs/msg/Centerline.msg" "geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg" NAME_WE)
add_custom_target(_racecar_cone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "racecar_cone_msgs" "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg" "geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Cone.msg" NAME_WE)
add_custom_target(_racecar_cone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "racecar_cone_msgs" "/ws/src/racecar_cone_msgs/msg/Cone.msg" "geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg" NAME_WE)
add_custom_target(_racecar_cone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "racecar_cone_msgs" "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg" "racecar_cone_msgs/Cone:geometry_msgs/Point:racecar_cone_msgs/ConePair:std_msgs/Header"
)

get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg" NAME_WE)
add_custom_target(_racecar_cone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "racecar_cone_msgs" "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg" "std_msgs/Header"
)

get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConeArray.msg" NAME_WE)
add_custom_target(_racecar_cone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "racecar_cone_msgs" "/ws/src/racecar_cone_msgs/msg/ConeArray.msg" "geometry_msgs/Point:racecar_cone_msgs/Cone:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePair.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_cpp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Centerline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_cpp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_cpp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_cpp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg"
  "${MSG_I_FLAGS}"
  "/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/ConePair.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_cpp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_cpp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConeArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(racecar_cone_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(racecar_cone_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(racecar_cone_msgs_generate_messages racecar_cone_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePair.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_cpp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Centerline.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_cpp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_cpp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_cpp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_cpp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_cpp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConeArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_cpp _racecar_cone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecar_cone_msgs_gencpp)
add_dependencies(racecar_cone_msgs_gencpp racecar_cone_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecar_cone_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePair.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_eus(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Centerline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_eus(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_eus(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_eus(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg"
  "${MSG_I_FLAGS}"
  "/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/ConePair.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_eus(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_eus(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConeArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(racecar_cone_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(racecar_cone_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(racecar_cone_msgs_generate_messages racecar_cone_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePair.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_eus _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Centerline.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_eus _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_eus _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_eus _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_eus _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_eus _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConeArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_eus _racecar_cone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecar_cone_msgs_geneus)
add_dependencies(racecar_cone_msgs_geneus racecar_cone_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecar_cone_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePair.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_lisp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Centerline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_lisp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_lisp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_lisp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg"
  "${MSG_I_FLAGS}"
  "/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/ConePair.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_lisp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_lisp(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConeArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(racecar_cone_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(racecar_cone_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(racecar_cone_msgs_generate_messages racecar_cone_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePair.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_lisp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Centerline.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_lisp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_lisp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_lisp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_lisp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_lisp _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConeArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_lisp _racecar_cone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecar_cone_msgs_genlisp)
add_dependencies(racecar_cone_msgs_genlisp racecar_cone_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecar_cone_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePair.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_nodejs(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Centerline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_nodejs(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_nodejs(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_nodejs(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg"
  "${MSG_I_FLAGS}"
  "/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/ConePair.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_nodejs(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_nodejs(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConeArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(racecar_cone_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(racecar_cone_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(racecar_cone_msgs_generate_messages racecar_cone_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePair.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_nodejs _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Centerline.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_nodejs _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_nodejs _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_nodejs _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_nodejs _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_nodejs _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConeArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_nodejs _racecar_cone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecar_cone_msgs_gennodejs)
add_dependencies(racecar_cone_msgs_gennodejs racecar_cone_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecar_cone_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePair.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_py(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Centerline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_py(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_py(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_py(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg"
  "${MSG_I_FLAGS}"
  "/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/ConePair.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_py(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs
)
_generate_msg_py(racecar_cone_msgs
  "/ws/src/racecar_cone_msgs/msg/ConeArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/ws/src/racecar_cone_msgs/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(racecar_cone_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(racecar_cone_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(racecar_cone_msgs_generate_messages racecar_cone_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePair.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_py _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Centerline.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_py _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/MidpointArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_py _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_py _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConePairArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_py _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/TrackMetrics.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_py _racecar_cone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/ws/src/racecar_cone_msgs/msg/ConeArray.msg" NAME_WE)
add_dependencies(racecar_cone_msgs_generate_messages_py _racecar_cone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecar_cone_msgs_genpy)
add_dependencies(racecar_cone_msgs_genpy racecar_cone_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecar_cone_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecar_cone_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(racecar_cone_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(racecar_cone_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecar_cone_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(racecar_cone_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(racecar_cone_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecar_cone_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(racecar_cone_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(racecar_cone_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecar_cone_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(racecar_cone_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(racecar_cone_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecar_cone_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(racecar_cone_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(racecar_cone_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
