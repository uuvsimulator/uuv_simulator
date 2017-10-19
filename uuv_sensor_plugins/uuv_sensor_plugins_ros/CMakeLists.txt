cmake_minimum_required(VERSION 2.8.3)
project(uuv_sensor_plugins_ros)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  uuv_gazebo_plugins
  sensor_msgs
  geometry_msgs
  std_msgs
  roscpp
  gazebo_plugins
  uuv_sensor_plugins
  uuv_sensor_plugins_ros_msgs)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

set(UUV_SENSOR_ROS_PLUGINS_LIST "")

catkin_package(
  INCLUDE_DIRS include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES uuv_gazebo_ros_camera
    uuv_gazebo_ros_dvl
    uuv_gazebo_ros_imu
    uuv_gazebo_ros_magnetometer
    uuv_gazebo_ros_rpt
    uuv_gazebo_ros_subseapressure
  CATKIN_DEPENDS uuv_gazebo_plugins
    sensor_msgs
    uuv_sensor_plugins
    uuv_sensor_plugins_ros_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(uuv_gazebo_ros_camera src/UnderwaterCameraROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_camera ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_camera ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_camera)

add_library(uuv_gazebo_ros_dvl
  src/DVLROSPlugin.cc
  src/SwitchableROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_dvl ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_dvl ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_dvl)

add_library(uuv_gazebo_ros_imu
  src/IMUROSPlugin.cc
  src/SwitchableROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_imu ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_imu ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_imu)

add_library(uuv_gazebo_ros_magnetometer
  src/MagnetometerROSPlugin.cc
  src/SwitchableROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_magnetometer ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_magnetometer ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_magnetometer)

add_library(uuv_gazebo_ros_rpt
  src/RPTROSPlugin.cc
  src/SwitchableROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_rpt ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_rpt ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_rpt)

add_library(uuv_gazebo_ros_gps src/GPSROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_gps ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(uuv_gazebo_ros_gps ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_gps)

add_library(uuv_gazebo_ros_subseapressure
  src/SubseaPressureROSPlugin.cc
  src/SwitchableROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_subseapressure ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_subseapressure ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_subseapressure)

add_library(uuv_gazebo_ros_chemical_concentration
  src/CPCROSPlugin.cc
  src/SwitchableROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_chemical_concentration ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_chemical_concentration ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_chemical_concentration)

# Gazebo ROS plugin adaptations with switch service function
add_library(uuv_gazebo_ros_switchable_gpu_ray_sensor
  src/SwitchableROSGPURaySensor.cc
  src/SwitchableROSSensorPlugin.cc
  src/SwitchableROSPlugin.cc)
target_link_libraries(uuv_gazebo_ros_switchable_gpu_ray_sensor
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES})
add_dependencies(uuv_gazebo_ros_switchable_gpu_ray_sensor
  ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_switchable_gpu_ray_sensor)

# install plug-ins
install(TARGETS ${UUV_SENSOR_ROS_PLUGINS_LIST}
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
