cmake_minimum_required(VERSION 2.8.3)
project(uuv_nc_parser)

find_package(catkin REQUIRED COMPONENTS 
  std_msgs 
  geometry_msgs 
  message_generation
  gazebo_msgs
  rospy
  rostopic
  rosgraph
  roslib
  uuv_gazebo_ros_plugins_msgs)

catkin_python_setup()

add_service_files(
  FILES
  GetNCVariables.srv
  GetNCData.srv
  GetNCVelocity.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)

catkin_install_python(PROGRAMS scripts/connect_ocean_data.py
                               scripts/generate_nc_test_data.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
