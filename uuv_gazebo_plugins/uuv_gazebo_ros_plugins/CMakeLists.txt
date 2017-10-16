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

set(UUV_GAZEBO_ROS_PLUGINS_LIST "")

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
  accelerations_test_plugin
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
add_dependencies(fin_ros_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST fin_ros_plugin)

add_library(thruster_ros_plugin src/ThrusterROSPlugin.cc)
target_link_libraries(thruster_ros_plugin ${catkin_LIBRARIES})
add_dependencies(thruster_ros_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST thruster_ros_plugin)

add_library(underwater_object_ros_plugin src/UnderwaterObjectROSPlugin.cc)
target_link_libraries(underwater_object_ros_plugin ${catkin_LIBRARIES})
add_dependencies(underwater_object_ros_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST underwater_object_ros_plugin)

add_library(joint_state_publisher src/JointStatePublisher.cc)
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST joint_state_publisher)

add_library(accelerations_test_plugin src/AccelerationsTestPlugin.cc)
target_link_libraries(accelerations_test_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST accelerations_test_plugin)

add_library(linear_battery_ros_plugin src/LinearBatteryROSPlugin.cc)
target_link_libraries(linear_battery_ros_plugin LinearBatteryPlugin ${catkin_LIBRARIES})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST linear_battery_ros_plugin)

add_library(custom_battery_consumer_ros_plugin src/CustomBatteryConsumerROSPlugin.cc)
target_link_libraries(custom_battery_consumer_ros_plugin ${catkin_LIBRARIES})
list(APPEND UUV_GAZEBO_ROS_PLUGINS_LIST custom_battery_consumer_ros_plugin)

install(TARGETS ${UUV_GAZEBO_ROS_PLUGINS_LIST}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

# Install urdf files
install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
