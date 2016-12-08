cmake_minimum_required(VERSION 2.8.3)
project(uuv_auv_teleop)

find_package(catkin REQUIRED dynamic_reconfigure)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# make sure configure headers are built before any node using them
#add_dependencies(example_node ${PROJECT_NAME}_gencfg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/JoyAnglesThrustControl.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
