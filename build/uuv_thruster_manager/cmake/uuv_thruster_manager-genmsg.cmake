# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "uuv_thruster_manager: 0 messages, 4 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(uuv_thruster_manager_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv" NAME_WE)
add_custom_target(_uuv_thruster_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_thruster_manager" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv" NAME_WE)
add_custom_target(_uuv_thruster_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_thruster_manager" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv" NAME_WE)
add_custom_target(_uuv_thruster_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_thruster_manager" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv" ""
)

get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv" NAME_WE)
add_custom_target(_uuv_thruster_manager_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_thruster_manager" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_cpp(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_cpp(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_cpp(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_thruster_manager
)

### Generating Module File
_generate_module_cpp(uuv_thruster_manager
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_thruster_manager
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(uuv_thruster_manager_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(uuv_thruster_manager_generate_messages uuv_thruster_manager_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_cpp _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_cpp _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_cpp _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_cpp _uuv_thruster_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_thruster_manager_gencpp)
add_dependencies(uuv_thruster_manager_gencpp uuv_thruster_manager_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_thruster_manager_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_eus(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_eus(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_eus(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_thruster_manager
)

### Generating Module File
_generate_module_eus(uuv_thruster_manager
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_thruster_manager
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(uuv_thruster_manager_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(uuv_thruster_manager_generate_messages uuv_thruster_manager_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_eus _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_eus _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_eus _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_eus _uuv_thruster_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_thruster_manager_geneus)
add_dependencies(uuv_thruster_manager_geneus uuv_thruster_manager_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_thruster_manager_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_lisp(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_lisp(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_lisp(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_thruster_manager
)

### Generating Module File
_generate_module_lisp(uuv_thruster_manager
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_thruster_manager
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(uuv_thruster_manager_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(uuv_thruster_manager_generate_messages uuv_thruster_manager_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_lisp _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_lisp _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_lisp _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_lisp _uuv_thruster_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_thruster_manager_genlisp)
add_dependencies(uuv_thruster_manager_genlisp uuv_thruster_manager_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_thruster_manager_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_nodejs(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_nodejs(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_nodejs(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_thruster_manager
)

### Generating Module File
_generate_module_nodejs(uuv_thruster_manager
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_thruster_manager
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(uuv_thruster_manager_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(uuv_thruster_manager_generate_messages uuv_thruster_manager_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_nodejs _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_nodejs _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_nodejs _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_nodejs _uuv_thruster_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_thruster_manager_gennodejs)
add_dependencies(uuv_thruster_manager_gennodejs uuv_thruster_manager_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_thruster_manager_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_py(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_py(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_thruster_manager
)
_generate_srv_py(uuv_thruster_manager
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_thruster_manager
)

### Generating Module File
_generate_module_py(uuv_thruster_manager
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_thruster_manager
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(uuv_thruster_manager_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(uuv_thruster_manager_generate_messages uuv_thruster_manager_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterCurve.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_py _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/GetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_py _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/ThrusterManagerInfo.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_py _uuv_thruster_manager_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_thruster_manager/srv/SetThrusterManagerConfig.srv" NAME_WE)
add_dependencies(uuv_thruster_manager_generate_messages_py _uuv_thruster_manager_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_thruster_manager_genpy)
add_dependencies(uuv_thruster_manager_genpy uuv_thruster_manager_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_thruster_manager_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_thruster_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_thruster_manager
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(uuv_thruster_manager_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_thruster_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_thruster_manager
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(uuv_thruster_manager_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_thruster_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_thruster_manager
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(uuv_thruster_manager_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_thruster_manager)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_thruster_manager
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(uuv_thruster_manager_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_thruster_manager)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_thruster_manager\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_thruster_manager
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(uuv_thruster_manager_generate_messages_py std_msgs_generate_messages_py)
endif()
