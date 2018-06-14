# Install script for directory: /home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/amishsqrrob/uuv_simulator/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/amishsqrrob/uuv_simulator/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/amishsqrrob/uuv_simulator/install" TYPE PROGRAM FILES "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/amishsqrrob/uuv_simulator/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/amishsqrrob/uuv_simulator/install" TYPE PROGRAM FILES "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/amishsqrrob/uuv_simulator/install/setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/amishsqrrob/uuv_simulator/install" TYPE FILE FILES "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/setup.bash")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/amishsqrrob/uuv_simulator/install/setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/amishsqrrob/uuv_simulator/install" TYPE FILE FILES "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/setup.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/amishsqrrob/uuv_simulator/install/setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/amishsqrrob/uuv_simulator/install" TYPE FILE FILES "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/setup.zsh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/amishsqrrob/uuv_simulator/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/amishsqrrob/uuv_simulator/install" TYPE FILE FILES "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_msgs/msg" TYPE FILE FILES
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Trajectory.msg"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/TrajectoryPoint.msg"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/Waypoint.msg"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/msg/WaypointSet.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_msgs/srv" TYPE FILE FILES
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/AddWaypoint.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ClearWaypoints.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitCircularTrajectory.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitHelicalTrajectory.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointsFromFile.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetWaypoints.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoTo.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GoToIncremental.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/Hold.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/IsRunningTrajectory.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitWaypointSet.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/InitRectTrajectory.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/StartTrajectory.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToAutomatic.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SwitchToManual.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetPIDParams.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetPIDParams.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetSMControllerParams.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetSMControllerParams.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/SetMBSMControllerParams.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/GetMBSMControllerParams.srv"
    "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/srv/ResetController.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_msgs/cmake" TYPE FILE FILES "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/uuv_control_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/amishsqrrob/uuv_simulator/devel/.private/uuv_control_msgs/include/uuv_control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/amishsqrrob/uuv_simulator/devel/.private/uuv_control_msgs/share/roseus/ros/uuv_control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/amishsqrrob/uuv_simulator/devel/.private/uuv_control_msgs/share/common-lisp/ros/uuv_control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/amishsqrrob/uuv_simulator/devel/.private/uuv_control_msgs/share/gennodejs/ros/uuv_control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/amishsqrrob/uuv_simulator/devel/.private/uuv_control_msgs/lib/python2.7/dist-packages/uuv_control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/amishsqrrob/uuv_simulator/devel/.private/uuv_control_msgs/lib/python2.7/dist-packages/uuv_control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/uuv_control_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_msgs/cmake" TYPE FILE FILES "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/uuv_control_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_msgs/cmake" TYPE FILE FILES
    "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/uuv_control_msgsConfig.cmake"
    "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/catkin_generated/installspace/uuv_control_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_control_msgs" TYPE FILE FILES "/home/amishsqrrob/uuv_simulator/src/uuv_control/uuv_control_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/amishsqrrob/uuv_simulator/build/uuv_control_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
