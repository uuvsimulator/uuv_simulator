^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_sensor_ros_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.8 (2018-07-07)
------------------
* Replaced GPS frame id with the link used in URDF
  Signed-off-by: Guy Stoppi <gstoppi@clearpathrobotics.com>
* Flipped GPS latitude,longitude so they increase when going north/east respectively
  Signed-off-by: Guy Stoppi <gstoppi@clearpathrobotics.com>
* Contributors: Guy Stoppi, Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães

0.5.7 (2018-07-06)
------------------

0.5.6 (2018-07-06)
------------------
* Replaced GPS frame id with the link used in URDF
  Signed-off-by: Guy Stoppi <gstoppi@clearpathrobotics.com>
* Flipped GPS latitude,longitude so they increase when going north/east respectively
  Signed-off-by: Guy Stoppi <gstoppi@clearpathrobotics.com>
* Contributors: Guy Stoppi, Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães

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
* FIX Bump version for uuv_sensor_ros_plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Different noise models for the base plugins
* ADD Seed for the noise generator
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <Musa.Marcusso@de.bosch.com>
* FIX Sensor inertial tensor information
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Input arguments for the salinity computation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Salinity measurement output based on the particle concentration
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Not publishing sensor measurement while processing the plume
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* RM Temporarily switchable scanner
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Typo in pressure xacro file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Current velocity vector setup using ignition
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <Musa.Marcusso@de.bosch.com>
* ADD Update the calls for the new Gazebo 9 API
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE PoseGT snippet
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Generate topic only when subscribers are found
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Dependency to tf2_ros
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Pose GT to generate both pose_gt and pose_gt_ned topics
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Path to sensor meshes
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Output message including linear velocity for pose_gt
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Camera plugin to library list
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Method to publish sensor state in update function
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Unified underwater camera sensor plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Unified RPT sensor plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Unified IMU sensor plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Unified chemical particle concentration sensor plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD More snippet options for ENU and NED frame
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* MV Snippets for new sensor package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Unified magnetometer sensor
  Includes option to set local NED frame.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Option to generate a local NED frame to sensor
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Reimplementation of DVL sensor
  Adding altitude information and beam link pose to the output.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD New URDF snippets
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Modified pose_gt sensor based on gazebo_ros_pkgs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD New extension of Gazebo's GPS plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD New unified subsea pressure sensor
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD ROS plugin base classes for ModelPlugin and SensorPlugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* MV Gazebo Protobuf messages to new package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* MV Sensor meshes to new package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Unique sensor plugin package
  Generation of Gazebo messages can be enabled, ROS messages
  will always be generated by all sensor plugins.
  This will diminish the complexity of the sensor plugin structure.
  Option to use the static TF frame "world_ned" instead of the
  default "world" frame.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Marcusso Manhaes Musa Morena (CR/AEI), Musa Morena Marcusso Manhaes
