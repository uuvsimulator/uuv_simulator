cmake_minimum_required(VERSION 2.8.3)
project(uuv_evaluation)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/trajectory_eval.py
                               scripts/smac_wrapper.py
                               scripts/run_best_worst_comparison.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
