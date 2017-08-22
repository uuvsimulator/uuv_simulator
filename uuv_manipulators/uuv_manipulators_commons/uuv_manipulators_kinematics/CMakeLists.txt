cmake_minimum_required(VERSION 2.8.3)
project(uuv_manipulators_kinematics)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  control_toolbox
  rospy
  uuv_manipulators_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf)

find_package(orocos_kdl REQUIRED)

catkin_python_setup()

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/kinematics_service.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosunit REQUIRED)

  foreach(UNIT_TESTS
    test/test_arm_interface.py
    test/test_jacobian.py
    catkin_add_nosetests(${UNIT_TESTS}))
  endforeach()

  foreach(TEST_LAUNCHERS
    test/test_kinematic_interfaces.launch
    test/test_start_simulation.launch
    add_rostest(${TEST_LAUNCHERS}))
  endforeach()
endif()
