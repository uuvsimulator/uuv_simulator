cmake_minimum_required(VERSION 2.8.3)
project(uuv_evaluation)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  roslib
  rosbag
  tf
  uuv_trajectory_control)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/trajectory_eval.py
                               scripts/smac_wrapper.py
                               scripts/run_best_worst_comparison.py
                               scripts/trajectory_eval.py
                               scripts/smac
                               scripts/create_smac_config_files
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

install(PROGRAMS
    scripts/run_smac
    scripts/clean_port_locks
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  foreach(T
      test/test_all.launch      
      )
     add_rostest(${T})
  endforeach()

  foreach(UNIT_TESTS
    test/test_run_task.py
    test/test_evaluation_trajectory.py
    test/test_kpis.py
    catkin_add_nosetests(${UNIT_TESTS}))
  endforeach()
  
endif()