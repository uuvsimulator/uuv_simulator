cmake_minimum_required(VERSION 2.8.3)
project(uuv_gazebo_ros_plugins_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation)

add_message_files(
  FILES
  FloatStamped.msg
  UnderwaterObjectModel.msg
  ThrusterConversionFcn.msg
)

add_service_files(
  FILES
  SetThrusterEfficiency.srv
  SetThrusterState.srv
  GetThrusterEfficiency.srv
  GetThrusterState.srv
  SetUseGlobalCurrentVel.srv
  GetModelProperties.srv
  SetFloat.srv
  GetFloat.srv
  GetListParam.srv
  GetThrusterConversionFcn.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    message_runtime
)
