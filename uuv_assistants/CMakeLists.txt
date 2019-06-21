cmake_minimum_required(VERSION 2.8.3)
project(uuv_assistants)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf
  topic_tools)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    tf
    topic_tools
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(uuv_message_to_tf
  src/message_to_tf.cc
)

target_link_libraries(uuv_message_to_tf
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/publish_footprints.py
                               scripts/set_simulation_timer.py
                               scripts/publish_world_models.py
                               scripts/publish_vehicle_footprint.py
                               scripts/unpause_simulation.py
                               scripts/create_new_robot_model
                               scripts/create_thruster_manager_configuration
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

install(TARGETS uuv_message_to_tf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
