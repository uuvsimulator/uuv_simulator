^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_sensor_ros_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* 0.5.3
* UPDATE CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Version number in uuv_teleop
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* UPDATE Catkin packages format to 2
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

* FIX Version number in uuv_teleop
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* UPDATE Catkin packages format to 2
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.3 (2018-07-04)
------------------
* Merge pull request `#256 <https://github.com/uuvsimulator/uuv_simulator/issues/256>`_ from uuvsimulator/hotfix/add_changelogs
  Hotfix/add changelogs
* ADD CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães

0.5.1 (2018-07-03)
------------------
* Merge pull request `#253 <https://github.com/uuvsimulator/uuv_simulator/issues/253>`_ from uuvsimulator/feature/trondheim_integration
  Feature/trondheim integration
* FIX Bump version for uuv_sensor_ros_plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Different noise models for the base plugins
* ADD Seed for the noise generator
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <Musa.Marcusso@de.bosch.com>
* Merge pull request `#245 <https://github.com/uuvsimulator/uuv_simulator/issues/245>`_ from uuvsimulator/feature/salinity_sensor
  Feature/salinity sensor
* FIX Sensor inertial tensor information
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Input arguments for the salinity computation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Salinity measurement output based on the particle concentration
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#244 <https://github.com/uuvsimulator/uuv_simulator/issues/244>`_ from uuvsimulator/fix/auv_controller_logic
  Fix/auv controller logic
* FIX Not publishing sensor measurement while processing the plume
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#227 <https://github.com/uuvsimulator/uuv_simulator/issues/227>`_ from uuvsimulator/fix/auv_traj_control
  Fix/auv traj control
* RM Temporarily switchable scanner
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Typo in pressure xacro file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#212 <https://github.com/uuvsimulator/uuv_simulator/issues/212>`_ from uuvsimulator/fix/gazebo_9_compatibility
  Fix/gazebo 9 compatibility
* FIX Current velocity vector setup using ignition
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <Musa.Marcusso@de.bosch.com>
* ADD Update the calls for the new Gazebo 9 API
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#193 <https://github.com/uuvsimulator/uuv_simulator/issues/193>`_ from uuvsimulator/feature/control_wrt_ned_frame
  Feature/control wrt ned frame
* CHANGE PoseGT snippet
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Generate topic only when subscribers are found
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Dependency to tf2_ros
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Pose GT to generate both pose_gt and pose_gt_ned topics
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#190 <https://github.com/uuvsimulator/uuv_simulator/issues/190>`_ from uuvsimulator/feature/sensor_with_ned_static_ref
  Feature/sensor with ned static ref
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
* Contributors: Marcusso Manhaes Musa Morena (CR/AEI), Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães
