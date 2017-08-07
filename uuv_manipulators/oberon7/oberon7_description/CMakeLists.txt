cmake_minimum_required(VERSION 2.8.3)
project(oberon7_description)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED) # this is only required to get gazebo's directories

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_descriptions
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY meshes params robots urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
