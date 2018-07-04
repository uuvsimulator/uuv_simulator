^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_manipulators_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* UPDATE Catkin packages format to 2
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.3 (2018-07-04)
------------------
* Merge pull request `#256 <https://github.com/uuvsimulator/uuv_simulator/issues/256>`_ from uuvsimulator/hotfix/add_changelogs
  Hotfix/add changelogs
* 0.5.2
* ADD CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães

0.5.1 (2018-07-03)
------------------
* Merge pull request `#253 <https://github.com/uuvsimulator/uuv_simulator/issues/253>`_ from uuvsimulator/feature/trondheim_integration
  Feature/trondheim integration
* CHANGE Bump version to 0.5.2
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#223 <https://github.com/uuvsimulator/uuv_simulator/issues/223>`_ from uuvsimulator/feature/auv_control_allocation
  Feature/auv control allocation
* CHANGE Version
* Merge pull request `#219 <https://github.com/uuvsimulator/uuv_simulator/issues/219>`_ from uuvsimulator/dev/fiducials
  FIX Use FIDUALS-ARUCO instead of RCARS-APRILTAGS
* FIX Use FIDUALS-ARUCO instead of RCARS-APRILTAGS
  Signed-off-by: Luiz Ricardo Douat <LuizRicardo.Doaut@de.bosch.com>
* Merge pull request `#182 <https://github.com/uuvsimulator/uuv_simulator/issues/182>`_ from uuvsimulator/fix/gm_process_random_seed
  Fix/gm process random seed
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#173 <https://github.com/uuvsimulator/uuv_simulator/issues/173>`_ from uuvsimulator/fix/travis_build_tests
  Fix/travis build tests
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#104 <https://github.com/uuvsimulator/uuv_simulator/issues/104>`_ from uuvsimulator/dev/travis_integration
  Dev/travis integration
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#103 <https://github.com/uuvsimulator/uuv_simulator/issues/103>`_ from uuvsimulator/dev/manipulator_eval
  Dev/manipulator eval
* FIX Test for ROS parameter of publish rate.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Resolving conflicts
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#86 <https://github.com/uuvsimulator/uuv_simulator/issues/86>`_ from uuvsimulator/dev/oberon7
  Dev/oberon7
* FIX Computation of next pose goal in Cartesian controller.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#49 <https://github.com/uuvsimulator/uuv_simulator/issues/49>`_ from uuvsimulator/fix/cart_controller_command_input
  FIX Index of the input command vector.
* FIX Index of the input command vector.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#42 <https://github.com/uuvsimulator/uuv_simulator/issues/42>`_ from uuvsimulator/dev/cart_control_vel_reference
  Dev/cart control vel reference
* ADD Exception handler for the case the joystick has been initialized with the wrong mapping.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Publish the current pose reference.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* RM Joystick subscriber
  ADD Velocity reference topic as controller input
  ADD Generation of reference topic and visual markers
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge branch 'master' of https://github.com/uuvsimulator/uuv_simulator
* Merge pull request `#39 <https://github.com/uuvsimulator/uuv_simulator/issues/39>`_ from uuvsimulator/dev/manipulator_jacobian
  Dev/manipulator jacobian
* FIX CMakeLists files
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#36 <https://github.com/uuvsimulator/uuv_simulator/issues/36>`_ from uuvsimulator/dev/logitech_teleop
  Dev/logitech teleop
* CHANGE Exception handler in joystick parsing
  callback function
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Modifying launch files and adding new demos with a joystick mapping for the Logitech Extreme 3D Pro.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#21 <https://github.com/uuvsimulator/uuv_simulator/issues/21>`_ from uuvsimulator/dev/manipulation
  Dev/manipulation
* Corrections to improve stability.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Correcting the exception handler.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adding new joint control node.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Modifying the controller nodes to parse the joystick input. Adding exclusion buttons to allow controlling a single arm at time.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#1 <https://github.com/uuvsimulator/uuv_simulator/issues/1>`_ from uuvsimulator/devel
  Initial public release
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Luiz Ricardo Douat, Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer (CR/AEI), lurido, sebastianscherer
