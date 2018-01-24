cmake_minimum_required(VERSION 2.8.3)
project(uuv_control_utils)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/set_timed_current_perturbation.py
                               scripts/apply_body_wrench.py
                               scripts/disturbance_manager.py
                               scripts/start_helical_trajectory.py
                               scripts/start_circular_trajectory.py
                               scripts/set_thruster_state.py
                               scripts/trajectory_marker_publisher.py
                               scripts/set_thruster_output_efficiency.py
                               scripts/send_waypoint_file.py
                               scripts/set_gm_current_perturbation.py
                               scripts/set_scalar_parameter.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
