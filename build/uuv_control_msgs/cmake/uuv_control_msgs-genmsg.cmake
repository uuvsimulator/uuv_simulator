# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "uuv_control_msgs: 4 messages, 22 services")

set(MSG_I_FLAGS "-Iuuv_control_msgs:/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(uuv_control_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv" "geometry_msgs/Point:std_msgs/Time:std_msgs/Header:uuv_control_msgs/Waypoint"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv" "std_msgs/Time:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg" "geometry_msgs/Accel:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:geometry_msgs/Point:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv" "std_msgs/Time:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg" "geometry_msgs/Accel:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Vector3:geometry_msgs/Pose:uuv_control_msgs/TrajectoryPoint"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv" "uuv_control_msgs/Waypoint:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv" "uuv_control_msgs/Waypoint:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv" "std_msgs/Time:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv" "uuv_control_msgs/Waypoint:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv" "std_msgs/Time:std_msgs/String"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg" "std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg" NAME_WE)
add_custom_target(_uuv_control_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_control_msgs" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg" "geometry_msgs/Point:std_msgs/Time:std_msgs/Header:uuv_control_msgs/Waypoint"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)

### Generating Services
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_cpp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
)

### Generating Module File
_generate_module_cpp(uuv_control_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(uuv_control_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(uuv_control_msgs_generate_messages uuv_control_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_cpp _uuv_control_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_control_msgs_gencpp)
add_dependencies(uuv_control_msgs_gencpp uuv_control_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_control_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)

### Generating Services
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_eus(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
)

### Generating Module File
_generate_module_eus(uuv_control_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(uuv_control_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(uuv_control_msgs_generate_messages uuv_control_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_eus _uuv_control_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_control_msgs_geneus)
add_dependencies(uuv_control_msgs_geneus uuv_control_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_control_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)

### Generating Services
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_lisp(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
)

### Generating Module File
_generate_module_lisp(uuv_control_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(uuv_control_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(uuv_control_msgs_generate_messages uuv_control_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_lisp _uuv_control_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_control_msgs_genlisp)
add_dependencies(uuv_control_msgs_genlisp uuv_control_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_control_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)

### Generating Services
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_nodejs(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
)

### Generating Module File
_generate_module_nodejs(uuv_control_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(uuv_control_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(uuv_control_msgs_generate_messages uuv_control_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_nodejs _uuv_control_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_control_msgs_gennodejs)
add_dependencies(uuv_control_msgs_gennodejs uuv_control_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_control_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_msg_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)

### Generating Services
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Time.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv"
  "${MSG_I_FLAGS}"
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)
_generate_srv_py(uuv_control_msgs
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
)

### Generating Module File
_generate_module_py(uuv_control_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(uuv_control_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(uuv_control_msgs_generate_messages uuv_control_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg" NAME_WE)
add_dependencies(uuv_control_msgs_generate_messages_py _uuv_control_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_control_msgs_genpy)
add_dependencies(uuv_control_msgs_genpy uuv_control_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_control_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_control_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(uuv_control_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(uuv_control_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_control_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(uuv_control_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(uuv_control_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_control_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(uuv_control_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(uuv_control_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_control_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(uuv_control_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(uuv_control_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_control_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(uuv_control_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(uuv_control_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
