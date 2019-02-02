cmake_minimum_required(VERSION 2.8.3)
project(uuv_control_cascaded_pid)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/AccelerationControl.cfg
  cfg/PositionControl.cfg
  cfg/VelocityControl.cfg
)

# make sure configure headers are built before any node using them
#add_dependencies(example_node ${PROJECT_NAME}_gencfg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/AccelerationControl.py
                               scripts/PositionControl.py
                               scripts/VelocityControl.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
