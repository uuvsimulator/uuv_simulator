cmake_minimum_required(VERSION 2.8.3)
project(uuv_gazebo_ros_plugins)

find_package(catkin REQUIRED COMPONENTS
  uuv_gazebo_plugins
  sensor_msgs
  geometry_msgs
  std_msgs
  uuv_gazebo_ros_plugins_msgs
  roscpp
  visualization_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
 INCLUDE_DIRS include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
 LIBRARIES fin_ros_plugin
  thruster_ros_plugin
  underwater_object_ros_plugin
  joint_state_publisher
 CATKIN_DEPENDS uuv_gazebo_plugins
  sensor_msgs
  geometry_msgs
  std_msgs
  uuv_gazebo_ros_plugins_msgs
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(fin_ros_plugin src/FinROSPlugin.cc)
target_link_libraries(fin_ros_plugin ${catkin_LIBRARIES})
add_dependencies(fin_ros_plugin
  ${catkin_EXPORTED_TARGETS})

add_library(thruster_ros_plugin src/ThrusterROSPlugin.cc)
target_link_libraries(thruster_ros_plugin ${catkin_LIBRARIES})
add_dependencies(thruster_ros_plugin
  ${catkin_EXPORTED_TARGETS})

add_library(underwater_object_ros_plugin src/UnderwaterObjectROSPlugin.cc)
target_link_libraries(underwater_object_ros_plugin ${catkin_LIBRARIES})
add_dependencies(underwater_object_ros_plugin 
  ${catkin_EXPORTED_TARGETS})

add_library(joint_state_publisher src/JointStatePublisher.cc)
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES})

add_library(accelerations_test_plugin
  src/AccelerationsTestPlugin.cc
)
target_link_libraries(accelerations_test_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS fin_ros_plugin thruster_ros_plugin underwater_object_ros_plugin joint_state_publisher accelerations_test_plugin
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
