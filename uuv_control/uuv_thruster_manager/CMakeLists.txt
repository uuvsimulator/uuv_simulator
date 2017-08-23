cmake_minimum_required(VERSION 2.8.3)
project(uuv_thruster_manager)

find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  message_generation
  tf
  rospy
  uuv_gazebo_ros_plugins_msgs
  geometry_msgs)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetThrusterCurve.srv
  ThrusterManagerInfo.srv
  SetThrusterManagerConfig.srv
  GetThrusterManagerConfig.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS scripts/thruster_allocator.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
