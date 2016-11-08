cmake_minimum_required(VERSION 2.8.3)
project(uuv_filtered_joy)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/uuv_joy_reference_models/default_joy_reference_model.py
                               scripts/uuv_joy_reference_models/filtered_joy_reference_model.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
