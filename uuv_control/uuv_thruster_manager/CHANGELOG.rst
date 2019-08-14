^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_thruster_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.13 (2019-08-12)
-------------------
* Add test dependencies
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add rostests
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add unit tests
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Update license header
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Update license header
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Update license header and add docstrings
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fix indentation of docstring
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add docstrings
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add test TAM configuration
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add mock robot description for thruster manager tests
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add mock configuration for thruster manager tests
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fixing Python 3 compatibility issues for scripts
  Related to issue `#268 <https://github.com/uuvsimulator/uuv_simulator/issues/268>`_
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.12 (2019-05-23)
-------------------

0.6.11 (2019-03-21)
-------------------

0.6.10 (2019-02-28)
-------------------
* Fix errors from catkin_lint
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.6.9 (2019-02-26)
------------------

0.6.8 (2019-02-14)
------------------

0.6.7 (2019-02-13)
------------------

0.6.6 (2019-02-12)
------------------

0.6.5 (2019-02-07)
------------------

0.6.4 (2019-02-03)
------------------
* FIX Dependency errors for ROS Buildfarm
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
  
0.6.3 (2018-12-13)
------------------
* CHANGE Use lowercase strings for e-mail
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* - Update thruster manager python to parse joint axes from urdf string via robot_description
  - Update gazebo ThrusterPlugin to get the thruster joint axis from the Joint API instead of hard coding it to be the x-axis
  Signed-off-by: Jordan Lack <jlack1987@gmail.com>
* Contributors: Jordan Lack, Musa Morena Marcusso Manhães

0.6.2 (2018-12-03)
------------------
* FIX rospy dependency
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX rospy dependencies
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.1 (2018-08-03)
------------------

0.6.0 (2018-07-31)
------------------

0.5.13 (2018-07-24)
-------------------

0.5.12 (2018-07-23)
-------------------

0.5.11 (2018-07-21)
-------------------

0.5.10 (2018-07-10)
-------------------

0.5.9 (2018-07-09)
------------------

0.5.8 (2018-07-07)
------------------

0.5.7 (2018-07-06)
------------------

0.5.6 (2018-07-06)
------------------

0.5.5 (2018-07-05)
------------------
* RM Merge messages from the change log
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* UPDATE Catkin packages format to 2
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.4 (2018-07-04)
------------------
* UPDATE Catkin packages format to 2
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.3 (2018-07-04)
------------------
* ADD CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.1 (2018-07-03)
------------------
* CHANGE Bump version to 0.5.2
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Input for thrust force information for compuation of command value
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Remove prints from thruster allocator
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Test for invalid position and rotation inputs
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Version
* ADD Subscriber for thruster_manager that is frame_id dependent
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Catkin requirements for catkin_make and catkin build
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Add message generation dependency
  Signed-off-by: Will Baker <willcbaker@gmail.com>
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX RexROV thruster manager configuration
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX TF listener timeout before TF frames are updated
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* remove tf listener where not required
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Increasing thruster manager timeout while waiting for the /tf messages for computation of TAM
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fixing the entry for max_thrust in the thruster manager. Now a list of max_thrust for each thruster can also be given.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fixing the thruster manager to accept multiple thruster models.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* install subdirectories in modules via setup.py
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* fix several files not being installed (can now source install/setup.bash)
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer, Sebastian Scherer (CR/AEI), Will Baker
