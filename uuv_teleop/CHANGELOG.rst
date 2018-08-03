^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Version number in uuv_teleop
Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* UPDATE Catkin packages format to 2
Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.4 (2018-07-04)
------------------
* FIX Version number in uuv_teleop
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* UPDATE Catkin packages format to 2
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.3 (2018-07-04)
------------------
* ADD CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.1 (2018-07-03)
------------------
* CHANGE Bump version to 0.5.2
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Dead zone for the axis input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Version
* FIX Catkin requirements for catkin_make and catkin build
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Allow only positive thrust
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Joy input gain for thruster command
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Default gain for each axis in teleop
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Default topic name for the thruster command in the AUV teleop launch file.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Exception handler for the case the joystick has been initialized with the wrong mapping.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* MV Teleop nodes to the uuv_teleop package.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* RM Move all teleop node files to uuv_teleop
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* RM Python module setup.py for AUV teleop package.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Catch exception and display error message when the current joystick mapping is wrong.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Modifying launch files and adding new demos with a joystick mapping for the Logitech Extreme 3D Pro.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* vehicle_teleop: use multiple axes for the same direction
  but with different magnitudes
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Adding a axis dead-zone for the vehicle teleop parser
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Changing the axis input gains for the vehicle teleop node.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* New vehicle teleop that allows blocking the vehicle command with joystick buttons.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Correcting the update of the publish time stamp. Publishing the wrench messages.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Restructuring the joystick control node for finned vehicles to receive configuration parameters through the parameter server
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer (CR/AEI)
