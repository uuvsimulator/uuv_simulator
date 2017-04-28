cmake_minimum_required(VERSION 2.8.3)
project(uuv_tutorial_seabed_world)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED) # this is only required to get gazebo's directories

catkin_package()

# Install models and world descriptions
add_subdirectory(world_models)
add_subdirectory(worlds)

# Install launch files
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN "*~" EXCLUDE)