cmake_minimum_required(VERSION 2.8.3)
find_package(catkin REQUIRED)

set(PACKAGE_NAME rexrov)

install(DIRECTORY mesh robots urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models/${PACKAGE_NAME}
        PATTERN "*~" EXCLUDE)
