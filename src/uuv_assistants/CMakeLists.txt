cmake_minimum_required(VERSION 2.8.3)
project(uuv_assistants)

find_package(catkin REQUIRED COMPONENTS
        rospy
        rostopic
        rosgraph
        nav_msgs
        geometry_msgs
        gazebo_msgs
        visualization_msgs
        tf)

catkin_package()

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
