cmake_minimum_required(VERSION 2.8.3)
project(orion_7p_description)

find_package(catkin REQUIRED COMPONENTS gazebo_plugins)

catkin_package()

install(DIRECTORY meshes urdf params robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
