cmake_minimum_required(VERSION 2.8.3)
project(uuv_sensor_ros_plugins)

add_definitions(-std=c++11)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED
  uuv_gazebo_plugins  
  sensor_msgs
  geometry_msgs
  std_msgs
  roscpp
  tf
  gazebo_plugins
  uuv_sensor_plugins_ros_msgs
  message_filters
  tf2_ros
  tf2)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(OGRE REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(UUV_SENSOR_ROS_PLUGINS_LIST "")

# Add Gazebo custom protobuf messages
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

message(STATUS "GAZEBO_MSG_INCLUDE_DIRS = " ${GAZEBO_MSG_INCLUDE_DIRS})

file(GLOB msgs msgs/*.proto)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(uuv_sensor_gazebo_msgs SHARED ${PROTO_SRCS})
target_link_libraries(uuv_sensor_gazebo_msgs ${PROTOBUF_LIBRARY} ${GAZEBO_LIBRARIES})

list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_sensor_gazebo_msgs)

file(COPY ${CMAKE_CURRENT_BINARY_DIR}/ DESTINATION ${CATKIN_DEVEL_PREFIX}/include/${PACKAGE_NAME} FILES_MATCHING PATTERN "*.pb.*")

catkin_package(
  INCLUDE_DIRS include
    ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
    ${GAZEBO_MSG_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OGRE_Paging_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  DEPENDS Eigen3 gazebo
  CATKIN_DEPENDS
    uuv_gazebo_plugins
    sensor_msgs
    uuv_sensor_plugins_ros_msgs
)

###########
## Build ##
###########

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

###############################################################################

add_library(uuv_gazebo_ros_base_model_plugin
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_base_model_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_base_model_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_base_model_plugin)

###############################################################################

add_library(uuv_gazebo_ros_base_sensor_plugin
  src/ROSBasePlugin.cc
  src/ROSBaseSensorPlugin.cc)
target_link_libraries(uuv_gazebo_ros_base_sensor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_base_sensor_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_base_sensor_plugin)

###############################################################################

add_library(uuv_gazebo_ros_gps_plugin
  src/GPSROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseSensorPlugin.cc)
target_link_libraries(uuv_gazebo_ros_gps_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_gps_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_gps_plugin)

###############################################################################

add_library(uuv_gazebo_ros_pose_gt_plugin
  src/PoseGTROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_pose_gt_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_pose_gt_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_pose_gt_plugin)

###############################################################################

add_library(uuv_gazebo_ros_subsea_pressure_plugin
  src/SubseaPressureROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_subsea_pressure_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_subsea_pressure_plugin uuv_sensor_gazebo_msgs ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_subsea_pressure_plugin)

###############################################################################

add_library(uuv_gazebo_ros_dvl_plugin
  src/DVLROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_dvl_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_dvl_plugin uuv_sensor_gazebo_msgs ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_dvl_plugin)

###############################################################################

add_library(uuv_gazebo_ros_magnetometer_plugin
  src/MagnetometerROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_magnetometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_magnetometer_plugin uuv_sensor_gazebo_msgs ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_magnetometer_plugin)

###############################################################################

add_library(uuv_gazebo_ros_cpc_plugin
  src/CPCROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_cpc_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_cpc_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_cpc_plugin)

###############################################################################

add_library(uuv_gazebo_ros_imu_plugin
  src/IMUROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_imu_plugin uuv_sensor_gazebo_msgs ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_imu_plugin)

###############################################################################

add_library(uuv_gazebo_ros_rpt_plugin
  src/RPTROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_rpt_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_rpt_plugin uuv_sensor_gazebo_msgs ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_rpt_plugin)

###############################################################################

add_library(uuv_gazebo_ros_camera_plugin src/UnderwaterCameraROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_camera_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_camera_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_camera_plugin)

#############
## Install ##
#############

install(
  TARGETS ${UUV_SENSOR_ROS_PLUGINS_LIST}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)

# Install mesh files
install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
