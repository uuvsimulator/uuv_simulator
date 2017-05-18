cmake_minimum_required(VERSION 2.8.3)
project(uuv_assistants)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/publish_footprints.py
                               scripts/set_simulation_timer.py
                               scripts/publish_world_models.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
