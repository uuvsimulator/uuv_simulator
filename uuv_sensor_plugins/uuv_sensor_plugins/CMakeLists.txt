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
find_package(Protobuf REQUIRED)

add_definitions(-std=c++11)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/msgs)

# add_subdirectory(msgs)
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
add_library(uuv_sensors_msgs SHARED ${PROTO_SRCS})
target_link_libraries(uuv_sensors_msgs ${PROTOBUF_LIBRARY} ${GAZEBO_LIBRARIES})

file(COPY ${CMAKE_CURRENT_BINARY_DIR}/ DESTINATION ${CATKIN_DEVEL_PREFIX}/include/${PACKAGE_NAME} FILES_MATCHING PATTERN "*.pb.*")

message(STATUS "Copying sensor message headers from = " ${CMAKE_CURRENT_BINARY_DIR} " to " ${CATKIN_DEVEL_PREFIX}/include/${PACKAGE_NAME})

catkin_package(
    INCLUDE_DIRS include
      ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
      ${GAZEBO_MSG_INCLUDE_DIRS}
      ${OGRE_INCLUDE_DIRS}
      ${OGRE_Paging_INCLUDE_DIRS}
    LIBRARIES uuv_gazebo_camera
      uuv_gazebo_dvl
      uuv_gazebo_imu
      uuv_gazebo_magnetometer
      uuv_gazebo_rpt
      uuv_gazebo_subseapressure
    DEPENDS Eigen3 gazebo
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
  ${EIGEN3_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

set(common_libraries
  uuv_sensors_msgs
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${SDF_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${OGRE_LIBRARIES}
  ${OGRE_Paging_LIBRARIES}
)

add_library(uuv_gazebo_camera SHARED
  src/SensorPlugin.cc
  src/UnderwaterCameraPlugin.cc
)
target_link_libraries(uuv_gazebo_camera ${common_libraries}
  gazebo_sensors DepthCameraPlugin)
add_dependencies(uuv_gazebo_camera uuv_sensors_msgs ${catkin_EXPORTED_TARGETS})

add_library(uuv_gazebo_dvl SHARED
  src/SensorPlugin.cc
  src/DvlPlugin.cc
)
target_link_libraries(uuv_gazebo_dvl ${common_libraries}
  gazebo_sensors)
add_dependencies(uuv_gazebo_dvl uuv_sensors_msgs ${catkin_EXPORTED_TARGETS})

add_library(uuv_gazebo_imu SHARED
  src/SensorPlugin.cc
  src/ImuPlugin.cc
)
target_link_libraries(uuv_gazebo_imu ${common_libraries})
add_dependencies(uuv_gazebo_imu uuv_sensors_msgs ${catkin_EXPORTED_TARGETS})

add_library(uuv_gazebo_magnetometer SHARED
  src/SensorPlugin.cc
  src/MagnetometerPlugin.cc
)
target_link_libraries(uuv_gazebo_magnetometer ${common_libraries})
add_dependencies(uuv_gazebo_magnetometer uuv_sensors_msgs ${catkin_EXPORTED_TARGETS})

add_library(uuv_gazebo_rpt SHARED
  src/SensorPlugin.cc
  src/RptPlugin.cc
)
target_link_libraries(uuv_gazebo_rpt ${common_libraries}
  gazebo_sensors)
add_dependencies(uuv_gazebo_rpt uuv_sensors_msgs ${catkin_EXPORTED_TARGETS})

add_library(uuv_gazebo_subseapressure SHARED
  src/SensorPlugin.cc
  src/SubseapressurePlugin.cc
)
target_link_libraries(uuv_gazebo_subseapressure ${common_libraries})
add_dependencies(uuv_gazebo_subseapressure uuv_sensors_msgs ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    uuv_gazebo_camera
    uuv_gazebo_dvl
    uuv_gazebo_imu
    uuv_gazebo_magnetometer
    uuv_gazebo_rpt
    uuv_gazebo_subseapressure
    uuv_sensors_msgs
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
install(DIRECTORY mesh
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
