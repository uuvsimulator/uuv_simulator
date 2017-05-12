cmake_minimum_required(VERSION 2.8.3)
project(uuv_smac_opt_nmb_sm_no_disturbances)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)