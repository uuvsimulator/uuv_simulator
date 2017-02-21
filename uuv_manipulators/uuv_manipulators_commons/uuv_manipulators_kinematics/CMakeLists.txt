cmake_minimum_required(VERSION 2.8.3)
project(uuv_manipulators_kinematics)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin)

find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/kinematics_service.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
