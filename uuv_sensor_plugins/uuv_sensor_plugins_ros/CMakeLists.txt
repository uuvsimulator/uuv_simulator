cmake_minimum_required(VERSION 2.8.3)
project(uuv_sensor_plugins_ros)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED gazebo_plugins sensor_msgs uuv_sensor_plugins
                             uuv_sensor_plugins_ros_msgs)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_sensor_plugins_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(uuv_gazebo_ros_camera src/UnderwaterCameraRosPlugin.cc)
target_link_libraries(uuv_gazebo_ros_camera ${catkin_LIBRARIES})
add_dependencies(uuv_gazebo_ros_camera uuv_gazebo_camera)

add_library(uuv_gazebo_ros_dvl src/DvlRosPlugin.cc)
target_link_libraries(uuv_gazebo_ros_dvl ${catkin_LIBRARIES})
add_dependencies(uuv_gazebo_ros_dvl
  uuv_gazebo_dvl ${catkin_EXPORTED_TARGETS})

add_library(uuv_gazebo_ros_imu src/ImuRosPlugin.cc)
target_link_libraries(uuv_gazebo_ros_imu ${catkin_LIBRARIES})
add_dependencies(uuv_gazebo_ros_imu uuv_gazebo_imu)

add_library(uuv_gazebo_ros_magnetometer src/MagnetometerRosPlugin.cc)
target_link_libraries(uuv_gazebo_ros_magnetometer ${catkin_LIBRARIES})
add_dependencies(uuv_gazebo_ros_magnetometer uuv_gazebo_magnetometer)

add_library(uuv_gazebo_ros_rpt src/RptRosPlugin.cc)
target_link_libraries(uuv_gazebo_ros_rpt ${catkin_LIBRARIES})
add_dependencies(uuv_gazebo_ros_rpt
  uuv_gazebo_rpt ${catkin_EXPORTED_TARGETS})

add_library(uuv_gazebo_ros_subseapressure src/SubseapressureRosPlugin.cc)
target_link_libraries(uuv_gazebo_ros_subseapressure ${catkin_LIBRARIES})
add_dependencies(uuv_gazebo_ros_subseapressure uuv_gazebo_subseapressure)

# install plug-ins
install(TARGETS uuv_gazebo_ros_camera uuv_gazebo_ros_dvl uuv_gazebo_ros_imu
    uuv_gazebo_ros_magnetometer uuv_gazebo_ros_rpt uuv_gazebo_ros_subseapressure
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install urdf files
install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
