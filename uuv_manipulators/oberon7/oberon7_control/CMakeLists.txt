cmake_minimum_required(VERSION 2.8.3)
project(oberon7_control)

find_package(catkin)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
