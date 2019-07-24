cmake_minimum_required(VERSION 2.8.3)
project(uuv_thruster_manager)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetThrusterCurve.srv
  ThrusterManagerInfo.srv
  SetThrusterManagerConfig.srv
  GetThrusterManagerConfig.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    std_msgs 
    message_runtime)

catkin_install_python(PROGRAMS scripts/thruster_allocator.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(T
      test/test_thrusters.test
      test/test_thruster_allocator_x_axis.test
      test/test_thruster_allocator_y_axis.test
      test/test_thruster_allocator_z_axis.test
      test/test_thruster_manager_proportional_correct.test
      )
     add_rostest(${T})
  endforeach()
endif()
