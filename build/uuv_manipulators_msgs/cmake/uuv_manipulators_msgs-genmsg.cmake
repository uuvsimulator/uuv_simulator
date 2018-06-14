# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "uuv_manipulators_msgs: 4 messages, 1 services")

set(MSG_I_FLAGS "-Iuuv_manipulators_msgs:/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(uuv_manipulators_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg" NAME_WE)
add_custom_target(_uuv_manipulators_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_manipulators_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg" NAME_WE)
add_custom_target(_uuv_manipulators_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_manipulators_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg" NAME_WE)
add_custom_target(_uuv_manipulators_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_manipulators_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg" "geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Wrench:geometry_msgs/Vector3:geometry_msgs/Point:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv" NAME_WE)
add_custom_target(_uuv_manipulators_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_manipulators_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point:sensor_msgs/JointState"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg" NAME_WE)
add_custom_target(_uuv_manipulators_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_manipulators_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_cpp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_cpp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_cpp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Services
_generate_srv_cpp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Module File
_generate_module_cpp(uuv_manipulators_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_manipulators_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(uuv_manipulators_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(uuv_manipulators_msgs_generate_messages uuv_manipulators_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_cpp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_cpp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_cpp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_cpp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_cpp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_manipulators_msgs_gencpp)
add_dependencies(uuv_manipulators_msgs_gencpp uuv_manipulators_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_manipulators_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_eus(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_eus(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_eus(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Services
_generate_srv_eus(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Module File
_generate_module_eus(uuv_manipulators_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_manipulators_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(uuv_manipulators_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(uuv_manipulators_msgs_generate_messages uuv_manipulators_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_eus _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_eus _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_eus _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_eus _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_eus _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_manipulators_msgs_geneus)
add_dependencies(uuv_manipulators_msgs_geneus uuv_manipulators_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_manipulators_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_lisp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_lisp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_lisp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Services
_generate_srv_lisp(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Module File
_generate_module_lisp(uuv_manipulators_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_manipulators_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(uuv_manipulators_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(uuv_manipulators_msgs_generate_messages uuv_manipulators_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_lisp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_lisp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_lisp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_lisp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_lisp _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_manipulators_msgs_genlisp)
add_dependencies(uuv_manipulators_msgs_genlisp uuv_manipulators_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_manipulators_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_nodejs(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_nodejs(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_nodejs(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Services
_generate_srv_nodejs(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Module File
_generate_module_nodejs(uuv_manipulators_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_manipulators_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(uuv_manipulators_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(uuv_manipulators_msgs_generate_messages uuv_manipulators_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_nodejs _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_nodejs _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_nodejs _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_nodejs _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_nodejs _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_manipulators_msgs_gennodejs)
add_dependencies(uuv_manipulators_msgs_gennodejs uuv_manipulators_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_manipulators_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_py(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_py(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_manipulators_msgs
)
_generate_msg_py(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Services
_generate_srv_py(uuv_manipulators_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_manipulators_msgs
)

### Generating Module File
_generate_module_py(uuv_manipulators_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_manipulators_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(uuv_manipulators_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(uuv_manipulators_msgs_generate_messages uuv_manipulators_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_py _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/ArmConfigCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_py _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndPointState.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_py _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/srv/SolveIK.srv" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_py _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_manipulators/uuv_manipulators_commons/uuv_manipulators_msgs/msg/EndeffectorCommand.msg" NAME_WE)
add_dependencies(uuv_manipulators_msgs_generate_messages_py _uuv_manipulators_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_manipulators_msgs_genpy)
add_dependencies(uuv_manipulators_msgs_genpy uuv_manipulators_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_manipulators_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_manipulators_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_manipulators_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(uuv_manipulators_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(uuv_manipulators_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(uuv_manipulators_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_manipulators_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_manipulators_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(uuv_manipulators_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(uuv_manipulators_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(uuv_manipulators_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_manipulators_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_manipulators_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(uuv_manipulators_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(uuv_manipulators_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(uuv_manipulators_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_manipulators_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_manipulators_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(uuv_manipulators_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(uuv_manipulators_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(uuv_manipulators_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_manipulators_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_manipulators_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_manipulators_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(uuv_manipulators_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(uuv_manipulators_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(uuv_manipulators_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
