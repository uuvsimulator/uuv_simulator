^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_thruster_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* 0.5.3
* UPDATE CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* UPDATE Catkin packages format to 2
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

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
* Merge pull request `#242 <https://github.com/uuvsimulator/uuv_simulator/issues/242>`_ from uuvsimulator/fix/thruster_model_conversion
  FIX Input for thrust force information for compuation of command value
* FIX Input for thrust force information for compuation of command value
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#224 <https://github.com/uuvsimulator/uuv_simulator/issues/224>`_ from uuvsimulator/feature/dubins_path
  Feature/dubins path
* CHANGE Remove prints from thruster allocator
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Test for invalid position and rotation inputs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#223 <https://github.com/uuvsimulator/uuv_simulator/issues/223>`_ from uuvsimulator/feature/auv_control_allocation
  Feature/auv control allocation
* CHANGE Version
* Merge pull request `#193 <https://github.com/uuvsimulator/uuv_simulator/issues/193>`_ from uuvsimulator/feature/control_wrt_ned_frame
  Feature/control wrt ned frame
* ADD Subscriber for thruster_manager that is frame_id dependent
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#191 <https://github.com/uuvsimulator/uuv_simulator/issues/191>`_ from uuvsimulator/fix/catkin_make_build
  FIX Catkin requirements for catkin_make and catkin build
* FIX Catkin requirements for catkin_make and catkin build
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#182 <https://github.com/uuvsimulator/uuv_simulator/issues/182>`_ from uuvsimulator/fix/gm_process_random_seed
  Fix/gm process random seed
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#179 <https://github.com/uuvsimulator/uuv_simulator/issues/179>`_ from willcbaker/fix/msg-install
  Add message generation dependency
* Add message generation dependency
  Signed-off-by: Will Baker <willcbaker@gmail.com>
* Merge pull request `#173 <https://github.com/uuvsimulator/uuv_simulator/issues/173>`_ from uuvsimulator/fix/travis_build_tests
  Fix/travis build tests
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#143 <https://github.com/uuvsimulator/uuv_simulator/issues/143>`_ from uuvsimulator/fix/rexrov_thruster_config
  FIX RexROV thruster manager configuration
* FIX RexROV thruster manager configuration
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#109 <https://github.com/uuvsimulator/uuv_simulator/issues/109>`_ from uuvsimulator/fix/thruster_manager_timeout
  FIX TF listener timeout before TF frames are updated
* FIX TF listener timeout before TF frames are updated
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#104 <https://github.com/uuvsimulator/uuv_simulator/issues/104>`_ from uuvsimulator/dev/travis_integration
  Dev/travis integration
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#68 <https://github.com/uuvsimulator/uuv_simulator/issues/68>`_ from uuvsimulator/dev/cpu_efficiency
  Dev/cpu efficiency
* remove tf listener where not required
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Merge pull request `#16 <https://github.com/uuvsimulator/uuv_simulator/issues/16>`_ from uuvsimulator/fix/thruster_manager_timeout
  Increasing thruster manager timeout while waiting for the /tf message…
* Increasing thruster manager timeout while waiting for the /tf messages for computation of TAM
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#15 <https://github.com/uuvsimulator/uuv_simulator/issues/15>`_ from uuvsimulator/fix/max_thrust
  Fixing the entry for max_thrust in the thruster manager. Now a list o…
* Fixing the entry for max_thrust in the thruster manager. Now a list of max_thrust for each thruster can also be given.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Merge pull request `#14 <https://github.com/uuvsimulator/uuv_simulator/issues/14>`_ from uuvsimulator/fix/multi_thruster_model
  Fixing the thruster manager to accept multiple thruster models.
* Fixing the thruster manager to accept multiple thruster models.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Merge pull request `#9 <https://github.com/uuvsimulator/uuv_simulator/issues/9>`_ from uuvsimulator/fix/catkin_install
  fix several files not being installed (can now source install/setup.bash)
* install subdirectories in modules via setup.py
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* fix several files not being installed (can now source install/setup.bash)
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Merge pull request `#1 <https://github.com/uuvsimulator/uuv_simulator/issues/1>`_ from uuvsimulator/devel
  Initial public release
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer, Sebastian Scherer (CR/AEI), Will Baker, lurido, sebastianscherer
