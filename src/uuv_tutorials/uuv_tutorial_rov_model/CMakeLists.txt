cmake_minimum_required(VERSION 2.8.3)
project(uuv_tutorial_rov_model)

find_package(catkin REQUIRED)

catkin_package()

set(PACKAGE_NAME tutorial_rov_model)

install(DIRECTORY mesh robots urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models/${PACKAGE_NAME}
        PATTERN "*~" EXCLUDE)