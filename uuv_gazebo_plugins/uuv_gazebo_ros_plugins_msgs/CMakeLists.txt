cmake_minimum_required(VERSION 2.8.3)
project(uuv_gazebo_ros_plugins_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs message_generation)

add_message_files(
  FILES
  FloatStamped.msg
  UnderwaterObjectModel.msg
)

add_service_files(
  FILES
  SetThrusterEfficiency.srv
  SetThrusterState.srv
  GetThrusterEfficiency.srv
  GetThrusterState.srv
  SetUseGlobalCurrentVel.srv
  GetModelProperties.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_gazebo_ros_plugins_msgs
   CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)
