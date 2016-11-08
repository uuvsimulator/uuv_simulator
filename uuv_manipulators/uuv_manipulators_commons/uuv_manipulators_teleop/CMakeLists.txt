cmake_minimum_required(VERSION 2.8.3)
project(uuv_manipulators_teleop)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    geometry_msgs
    cmake_modules)

catkin_python_setup()

catkin_package()


catkin_install_python(PROGRAMS
      scripts/ramp_generation.py
      scripts/cartesian_teleop.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
