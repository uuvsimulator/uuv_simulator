cmake_minimum_required(VERSION 2.8.3)
project(uuv_world_ros_plugins_msgs)

find_package(catkin REQUIRED
  message_generation
  std_msgs)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetCurrentModel.srv
  GetCurrentModel.srv
  SetCurrentVelocity.srv
  SetCurrentDirection.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
   CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS
)
