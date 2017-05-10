cmake_minimum_required(VERSION 2.8.3)
project(uuv_smac_opt_nmb_sm_timed_disturbances)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_smac_opt_pid_no_disturbances
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)