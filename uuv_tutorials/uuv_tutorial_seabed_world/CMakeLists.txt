cmake_minimum_required(VERSION 2.8.3)
project(uuv_tutorial_seabed_world)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED) # this is only required to get gazebo's directories

catkin_package()

install(DIRECTORY launch worlds models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
