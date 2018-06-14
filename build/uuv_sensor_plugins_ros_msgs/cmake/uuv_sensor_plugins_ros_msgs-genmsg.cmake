# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "uuv_sensor_plugins_ros_msgs: 6 messages, 1 services")

set(MSG_I_FLAGS "-Iuuv_sensor_plugins_ros_msgs:/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(uuv_sensor_plugins_ros_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg" NAME_WE)
add_custom_target(_uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_sensor_plugins_ros_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg" NAME_WE)
add_custom_target(_uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_sensor_plugins_ros_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg" NAME_WE)
add_custom_target(_uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_sensor_plugins_ros_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg" NAME_WE)
add_custom_target(_uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_sensor_plugins_ros_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg" "uuv_sensor_plugins_ros_msgs/PositionWithCovariance:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg" NAME_WE)
add_custom_target(_uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_sensor_plugins_ros_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv" NAME_WE)
add_custom_target(_uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_sensor_plugins_ros_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg" NAME_WE)
add_custom_target(_uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_sensor_plugins_ros_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg" "geometry_msgs/PoseStamped:std_msgs/Header:geometry_msgs/Quaternion:uuv_sensor_plugins_ros_msgs/DVLBeam:geometry_msgs/Point:geometry_msgs/Vector3:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_cpp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_cpp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_cpp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_cpp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_cpp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Services
_generate_srv_cpp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Module File
_generate_module_cpp(uuv_sensor_plugins_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(uuv_sensor_plugins_ros_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages uuv_sensor_plugins_ros_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_cpp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_cpp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_cpp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_cpp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_cpp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_cpp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_cpp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_sensor_plugins_ros_msgs_gencpp)
add_dependencies(uuv_sensor_plugins_ros_msgs_gencpp uuv_sensor_plugins_ros_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_sensor_plugins_ros_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_eus(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_eus(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_eus(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_eus(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_eus(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Services
_generate_srv_eus(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Module File
_generate_module_eus(uuv_sensor_plugins_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(uuv_sensor_plugins_ros_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages uuv_sensor_plugins_ros_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_eus _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_eus _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_eus _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_eus _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_eus _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_eus _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_eus _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_sensor_plugins_ros_msgs_geneus)
add_dependencies(uuv_sensor_plugins_ros_msgs_geneus uuv_sensor_plugins_ros_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_sensor_plugins_ros_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_lisp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_lisp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_lisp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_lisp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_lisp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Services
_generate_srv_lisp(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Module File
_generate_module_lisp(uuv_sensor_plugins_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(uuv_sensor_plugins_ros_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages uuv_sensor_plugins_ros_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_lisp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_lisp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_lisp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_lisp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_lisp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_lisp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_lisp _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_sensor_plugins_ros_msgs_genlisp)
add_dependencies(uuv_sensor_plugins_ros_msgs_genlisp uuv_sensor_plugins_ros_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_sensor_plugins_ros_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_nodejs(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_nodejs(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_nodejs(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_nodejs(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_nodejs(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Services
_generate_srv_nodejs(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Module File
_generate_module_nodejs(uuv_sensor_plugins_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(uuv_sensor_plugins_ros_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages uuv_sensor_plugins_ros_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_nodejs _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_nodejs _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_nodejs _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_nodejs _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_nodejs _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_nodejs _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_nodejs _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_sensor_plugins_ros_msgs_gennodejs)
add_dependencies(uuv_sensor_plugins_ros_msgs_gennodejs uuv_sensor_plugins_ros_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_sensor_plugins_ros_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_py(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_py(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_py(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_py(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)
_generate_msg_py(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Services
_generate_srv_py(uuv_sensor_plugins_ros_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
)

### Generating Module File
_generate_module_py(uuv_sensor_plugins_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(uuv_sensor_plugins_ros_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages uuv_sensor_plugins_ros_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/Salinity.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_py _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVLBeam.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_py _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/ChemicalParticleConcentration.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_py _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovarianceStamped.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_py _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/PositionWithCovariance.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_py _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/srv/ChangeSensorState.srv" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_py _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_sensor_plugins/uuv_sensor_plugins_ros_msgs/msg/DVL.msg" NAME_WE)
add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_py _uuv_sensor_plugins_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_sensor_plugins_ros_msgs_genpy)
add_dependencies(uuv_sensor_plugins_ros_msgs_genpy uuv_sensor_plugins_ros_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_sensor_plugins_ros_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_sensor_plugins_ros_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(uuv_sensor_plugins_ros_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
