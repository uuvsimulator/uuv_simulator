cmake_minimum_required(VERSION 2.8.3)
project(uuv_gazebo_plugins)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/msgs)

set(UUV_GAZEBO_PLUGINS_LIST "")

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
add_library(uuv_gazebo_plugins_msgs SHARED ${PROTO_SRCS})
target_link_libraries(uuv_gazebo_plugins_msgs ${PROTOBUF_LIBRARY} ${GAZEBO_LIBRARIES})
list(APPEND UUV_GAZEBO_PLUGINS_LIST uuv_gazebo_plugins_msgs)

catkin_package(
  INCLUDE_DIRS include
               ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
               ${PROJECT_BINARY_DIR}/msgs
               ${Boost_INCLUDE_DIR}
               ${catkin_INCLUDE_DIRS}
               ${GAZEBO_INCLUDE_DIRS}
               ${GAZEBO_MSG_INCLUDE_DIRS}
               ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES underwater_object_plugin thruster_plugin fin_plugin
  DEPENDS gazebo
)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${GAZEBO_MSG_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})

link_directories(${EIGEN3_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})

###########
## Build ##
###########

add_library(dynamics
    src/Dynamics.cc
)
list(APPEND UUV_GAZEBO_PLUGINS_LIST dynamics)

add_library(fin_plugin
    src/LiftDragModel.cc
    src/FinPlugin.cc
)
target_link_libraries(fin_plugin
    dynamics
    uuv_gazebo_plugins_msgs
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES})
add_dependencies(fin_plugin
  uuv_gazebo_plugins_msgs
  ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_GAZEBO_PLUGINS_LIST fin_plugin)

add_library(underwater_object_plugin
    src/HydrodynamicModel.cc
    src/BuoyantObject.cc
    src/UnderwaterObjectPlugin.cc
)
target_link_libraries(underwater_object_plugin
    ${GAZEBO_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)
list(APPEND UUV_GAZEBO_PLUGINS_LIST underwater_object_plugin)

add_library(thruster_plugin
    src/ThrusterConversionFcn.cc
    src/ThrusterPlugin.cc
)
target_link_libraries(thruster_plugin
    dynamics
    uuv_gazebo_plugins_msgs
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES})
add_dependencies(thruster_plugin
    uuv_gazebo_plugins_msgs
    ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_GAZEBO_PLUGINS_LIST thruster_plugin)

add_library(umbilical_plugin
  src/UmbilicalModel.cc
  src/UmbilicalPlugin.cc)
target_link_libraries(umbilical_plugin
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
list(APPEND UUV_GAZEBO_PLUGINS_LIST umbilical_plugin)

#############
## Install ##
#############

install(TARGETS ${UUV_GAZEBO_PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".hh"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(UNIT_ThrusterConversionFcn_TEST
    src/ThrusterConversionFcn_TEST.cc)
  target_link_libraries(UNIT_ThrusterConversionFcn_TEST
    thruster_plugin)

  catkin_add_gtest(UNIT_ThrusterDynamics_TEST
    src/ThrusterDynamics_TEST.cc)
  target_link_libraries(UNIT_ThrusterDynamics_TEST
    thruster_plugin ${GAZEBO_LIBRARIES})
endif()
