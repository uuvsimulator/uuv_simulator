cmake_minimum_required(VERSION 2.8.3)
project(uuv_manipulators_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  EndPointState.msg
  EndeffectorCommand.msg
  EndeffectorState.msg
  ArmConfigCommand.msg
)

add_service_files(DIRECTORY srv FILES
  SolveIK.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
)
