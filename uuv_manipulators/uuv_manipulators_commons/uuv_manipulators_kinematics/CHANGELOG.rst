^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_manipulators_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* ADD Functionalities for generation of Jacobian matrices
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#65 <https://github.com/uuvsimulator/uuv_simulator/issues/65>`_ from uuvsimulator/fix/catkin_tools_config
  Fix/catkin tools config
* Merge pull request `#64 <https://github.com/uuvsimulator/uuv_simulator/issues/64>`_ from uuvsimulator/fix/jacobian_config
  FIX PyKDL call for Jacobian computation.
* FIX PyKDL call for Jacobian computation.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Package configuration for catkin tools.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#60 <https://github.com/uuvsimulator/uuv_simulator/issues/60>`_ from uuvsimulator/fix/jacobian
  FIX Jacobian matrix null output.
* FIX Jacobian matrix null output.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge branch 'master' of https://github.com/uuvsimulator/uuv_simulator
* Merge pull request `#39 <https://github.com/uuvsimulator/uuv_simulator/issues/39>`_ from uuvsimulator/dev/manipulator_jacobian
  Dev/manipulator jacobian
* RM Redundant initialization of the KDL inverse kinematics solver.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Test for forward and inverse kinematics.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Move inverse kinematics method to the kinematic chain interface class.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Move the inverse kinematics solver to the arm interface.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* FIX CMakeLists files
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Option to generate Jacobian matrices different final joint entities.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Test units for the arm interface.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#21 <https://github.com/uuvsimulator/uuv_simulator/issues/21>`_ from uuvsimulator/dev/manipulation
  Dev/manipulation
* uuv_manipulator_interfaces: catch sqrt of 0 error
  Signed-off-by: Sebastian Scherer <Sebastian.Scherer2@de.bosch.com>
* Removing unused MimicJointPlugin.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adding the information for mimic joints (gripper control)
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#9 <https://github.com/uuvsimulator/uuv_simulator/issues/9>`_ from uuvsimulator/fix/catkin_install
  fix several files not being installed (can now source install/setup.bash)
* Correcting import of xml_reflection package.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* install subdirectories in modules via setup.py
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* fix several files not being installed (can now source install/setup.bash)
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Merge pull request `#1 <https://github.com/uuvsimulator/uuv_simulator/issues/1>`_ from uuvsimulator/devel
  Initial public release
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer, Sebastian Scherer (CR/AEI), lurido, sebastianscherer
