cmake_minimum_required(VERSION 2.8.3)
project(uuv_manipulators_control)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    uuv_manipulators_kinematics
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    uuv_manipulators_kinematics
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED system filesystem date_time thread)

link_directories(${catkin_LIBRARY_DIRS})

catkin_install_python(PROGRAMS
      scripts/set_joint_config.py
      scripts/jt_cartesian_controller.py
      scripts/gripper_controller.py
      scripts/joint_position_controller.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
