# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "uuv_auv_control_allocator: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iuuv_auv_control_allocator:/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(uuv_auv_control_allocator_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg" NAME_WE)
add_custom_target(_uuv_auv_control_allocator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uuv_auv_control_allocator" "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg" "geometry_msgs/Vector3:std_msgs/Header:geometry_msgs/Wrench"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(uuv_auv_control_allocator
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_auv_control_allocator
)

### Generating Services

### Generating Module File
_generate_module_cpp(uuv_auv_control_allocator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_auv_control_allocator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(uuv_auv_control_allocator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(uuv_auv_control_allocator_generate_messages uuv_auv_control_allocator_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg" NAME_WE)
add_dependencies(uuv_auv_control_allocator_generate_messages_cpp _uuv_auv_control_allocator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_auv_control_allocator_gencpp)
add_dependencies(uuv_auv_control_allocator_gencpp uuv_auv_control_allocator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_auv_control_allocator_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(uuv_auv_control_allocator
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_auv_control_allocator
)

### Generating Services

### Generating Module File
_generate_module_eus(uuv_auv_control_allocator
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_auv_control_allocator
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(uuv_auv_control_allocator_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(uuv_auv_control_allocator_generate_messages uuv_auv_control_allocator_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg" NAME_WE)
add_dependencies(uuv_auv_control_allocator_generate_messages_eus _uuv_auv_control_allocator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_auv_control_allocator_geneus)
add_dependencies(uuv_auv_control_allocator_geneus uuv_auv_control_allocator_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_auv_control_allocator_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(uuv_auv_control_allocator
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_auv_control_allocator
)

### Generating Services

### Generating Module File
_generate_module_lisp(uuv_auv_control_allocator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_auv_control_allocator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(uuv_auv_control_allocator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(uuv_auv_control_allocator_generate_messages uuv_auv_control_allocator_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg" NAME_WE)
add_dependencies(uuv_auv_control_allocator_generate_messages_lisp _uuv_auv_control_allocator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_auv_control_allocator_genlisp)
add_dependencies(uuv_auv_control_allocator_genlisp uuv_auv_control_allocator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_auv_control_allocator_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(uuv_auv_control_allocator
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_auv_control_allocator
)

### Generating Services

### Generating Module File
_generate_module_nodejs(uuv_auv_control_allocator
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_auv_control_allocator
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(uuv_auv_control_allocator_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(uuv_auv_control_allocator_generate_messages uuv_auv_control_allocator_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg" NAME_WE)
add_dependencies(uuv_auv_control_allocator_generate_messages_nodejs _uuv_auv_control_allocator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_auv_control_allocator_gennodejs)
add_dependencies(uuv_auv_control_allocator_gennodejs uuv_auv_control_allocator_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_auv_control_allocator_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(uuv_auv_control_allocator
  "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Wrench.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_auv_control_allocator
)

### Generating Services

### Generating Module File
_generate_module_py(uuv_auv_control_allocator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_auv_control_allocator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(uuv_auv_control_allocator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(uuv_auv_control_allocator_generate_messages uuv_auv_control_allocator_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_auv_control_allocator/msg/AUVCommand.msg" NAME_WE)
add_dependencies(uuv_auv_control_allocator_generate_messages_py _uuv_auv_control_allocator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uuv_auv_control_allocator_genpy)
add_dependencies(uuv_auv_control_allocator_genpy uuv_auv_control_allocator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uuv_auv_control_allocator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_auv_control_allocator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uuv_auv_control_allocator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(uuv_auv_control_allocator_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(uuv_auv_control_allocator_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_auv_control_allocator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uuv_auv_control_allocator
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(uuv_auv_control_allocator_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(uuv_auv_control_allocator_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_auv_control_allocator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uuv_auv_control_allocator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(uuv_auv_control_allocator_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(uuv_auv_control_allocator_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_auv_control_allocator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uuv_auv_control_allocator
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(uuv_auv_control_allocator_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(uuv_auv_control_allocator_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_auv_control_allocator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_auv_control_allocator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uuv_auv_control_allocator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(uuv_auv_control_allocator_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(uuv_auv_control_allocator_generate_messages_py geometry_msgs_generate_messages_py)
endif()
