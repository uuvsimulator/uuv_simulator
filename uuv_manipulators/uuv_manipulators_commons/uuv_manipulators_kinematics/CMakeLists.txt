cmake_minimum_required(VERSION 2.8.3)
project(uuv_manipulators_kinematics)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(MIMIC_JOINTS gazebo_ros_mimic_joints)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  control_toolbox)

find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package(
 INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
 LIBRARIES ${MIMIC_JOINTS}
 CATKIN_DEPENDS
    roscpp
    gazebo_ros
    control_toolbox
  DEPENDS
    python_orocos_kdl
    urdfdom
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

find_package(LAPACK REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

add_library(${MIMIC_JOINTS} src/MimicJointPlugin.cc)
target_link_libraries(${MIMIC_JOINTS} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${MIMIC_JOINTS}
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

catkin_install_python(PROGRAMS scripts/kinematics_service.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
