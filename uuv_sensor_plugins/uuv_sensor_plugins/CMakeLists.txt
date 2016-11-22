cmake_minimum_required(VERSION 2.8.3)
project(uuv_sensor_plugins)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

set (CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake CACHE PATH "Location of CMake scripts")

# include(${CMAKE_DIR}/FindOGRE.cmake)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(OGRE REQUIRED)
find_package(OpenCV REQUIRED)

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

add_definitions(-std=c++11)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/msgs)
catkin_package(
    INCLUDE_DIRS include ${PROJECT_BINARY_DIR}/msgs ${GAZEBO_MSG_INCLUDE_DIRS}
                 ${OGRE_INCLUDE_DIRS} ${OGRE_Paging_INCLUDE_DIRS}
    LIBRARIES uuv_gazebo_camera uuv_gazebo_dvl uuv_gazebo_imu
              uuv_gazebo_magnetometer uuv_gazebo_rpt uuv_gazebo_subseapressure
    DEPENDS Eigen3 gazebo
)

add_subdirectory(msgs)

set(common_libraries
  uuv_sensors_msgs
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES}
  ${SDF_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${OGRE_LIBRARIES} ${OGRE_Paging_LIBRARIES}
)

add_library(uuv_gazebo_camera SHARED
  src/SensorPlugin.cc
  src/UnderwaterCameraPlugin.cc
)
target_link_libraries(uuv_gazebo_camera ${common_libraries}
  gazebo_sensors DepthCameraPlugin)
add_dependencies(uuv_gazebo_camera uuv_sensors_msgs)

add_library(uuv_gazebo_dvl SHARED
  src/SensorPlugin.cc
  src/DvlPlugin.cc
)
target_link_libraries(uuv_gazebo_dvl ${common_libraries}
  gazebo_sensors)
add_dependencies(uuv_gazebo_dvl uuv_sensors_msgs)

add_library(uuv_gazebo_imu SHARED
  src/SensorPlugin.cc
  src/ImuPlugin.cc
)
target_link_libraries(uuv_gazebo_imu ${common_libraries})
add_dependencies(uuv_gazebo_imu uuv_sensors_msgs)

add_library(uuv_gazebo_magnetometer SHARED
  src/SensorPlugin.cc
  src/MagnetometerPlugin.cc
)
target_link_libraries(uuv_gazebo_magnetometer ${common_libraries})
add_dependencies(uuv_gazebo_magnetometer uuv_sensors_msgs)

add_library(uuv_gazebo_rpt SHARED
  src/SensorPlugin.cc
  src/RptPlugin.cc
)
target_link_libraries(uuv_gazebo_rpt ${common_libraries}
  gazebo_sensors)
add_dependencies(uuv_gazebo_rpt uuv_sensors_msgs)

add_library(uuv_gazebo_subseapressure SHARED
  src/SensorPlugin.cc
  src/SubseapressurePlugin.cc
)
target_link_libraries(uuv_gazebo_subseapressure ${common_libraries})
add_dependencies(uuv_gazebo_subseapressure uuv_sensors_msgs)

install(
  TARGETS
    uuv_gazebo_camera
    uuv_gazebo_dvl
    uuv_gazebo_imu
    uuv_gazebo_magnetometer
    uuv_gazebo_rpt
    uuv_gazebo_subseapressure
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install mesh files
install(DIRECTORY mesh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
