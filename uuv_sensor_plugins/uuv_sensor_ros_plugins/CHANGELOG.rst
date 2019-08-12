^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_sensor_ros_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.13 (2019-08-12)
-------------------

0.6.12 (2019-05-23)
-------------------

0.6.11 (2019-03-21)
-------------------
* Add new depends to catkin package
  Signed-off-by: Will Baker <wbaker@houstonmechatronics.com>
* Added fls implementation by Niels Bohr as explained in the following issue thread: https://github.com/uuvsimulator/uuv_simulator/issues/48, with the main code of the fls contained here: https://github.com/smarc-project/smarc_simulations/blob/master/smarc_gazebo_plugins/smarc_gazebo_ros_plugins/src/gazebo_ros_image_sonar.cpp. \n\n TODO: Modify FLS to be better representation (current FLS still abuses Depth cam), and integrate FLS into our project environment/requirement better
  Add depends to pacakge.xml
  # Conflicts:
  #	uuv_descriptions/robots/rexrov_test.xacro
  #	uuv_sensor_plugins/uuv_sensor_ros_plugins/CMakeLists.txt
  #	uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/sonar_snippets.xacro
  Signed-off-by: Will Baker <willcbaker@gmail.com>
* Contributors: Will Baker

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
* Fix copy Gazebo message files for ROS Buildfarm
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães

0.6.6 (2019-02-12)
------------------
* Fix gazebo_dev dependency
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.6.5 (2019-02-07)
------------------

0.6.4 (2019-02-03)
------------------
* FIX Dependency errors for ROS Buildfarm
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.3 (2018-12-13)
------------------
* CHANGE Use lowercase strings for e-mail
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Replace check for unit input for strings
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Unit test to test integrity of URDF files
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.2 (2018-12-03)
------------------
* CHANGE Name of the ROS sensor messages package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Name of the ROS sensor message package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Compilation configuration
  * Add COMPONENTS option to find_package(catkin) (fix catkin lint error)
  * Change name of `uuv_sensor_plugins_ros_msgs`  to `uuv_sensor_ros_plugins_msgs`
  * Move add library command for Gazebo Protobuf messages
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.1 (2018-08-03)
------------------

0.6.0 (2018-07-31)
------------------

0.5.13 (2018-07-24)
-------------------
* ADD UUV Gazebo ROS libraries to be exported
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.5.12 (2018-07-23)
-------------------

0.5.11 (2018-07-21)
-------------------
* FIX CMAKE_CXX_FLAGS in uuv_sensor_ros_plugins
  Signed-off-by: Gabriel Arjones <arjones@arjones.com>
* Install URDF folder
  Signed-off-by: Will Baker <wbaker@houstonmechatronics.com>
* Contributors: Gabriel Arjones, Will Baker

0.5.10 (2018-07-10)
-------------------

0.5.9 (2018-07-09)
------------------

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
Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Version number in uuv_teleop
Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* UPDATE Catkin packages format to 2
Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.4 (2018-07-04)
------------------
* FIX Version number in uuv_teleop
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
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
* FIX Bump version for uuv_sensor_ros_plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Different noise models for the base plugins
* ADD Seed for the noise generator
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <musa.marcusso@de.bosch.com>
* FIX Sensor inertial tensor information
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Input arguments for the salinity computation
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Salinity measurement output based on the particle concentration
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Not publishing sensor measurement while processing the plume
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Temporarily switchable scanner
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Typo in pressure xacro file
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Current velocity vector setup using ignition
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <musa.marcusso@de.bosch.com>
* ADD Update the calls for the new Gazebo 9 API
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE PoseGT snippet
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Generate topic only when subscribers are found
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Dependency to tf2_ros
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Pose GT to generate both pose_gt and pose_gt_ned topics
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Path to sensor meshes
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Output message including linear velocity for pose_gt
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Camera plugin to library list
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Method to publish sensor state in update function
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Unified underwater camera sensor plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Unified RPT sensor plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Unified IMU sensor plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Unified chemical particle concentration sensor plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD More snippet options for ENU and NED frame
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* MV Snippets for new sensor package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Unified magnetometer sensor
  Includes option to set local NED frame.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Option to generate a local NED frame to sensor
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Reimplementation of DVL sensor
  Adding altitude information and beam link pose to the output.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD New URDF snippets
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Modified pose_gt sensor based on gazebo_ros_pkgs
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD New extension of Gazebo's GPS plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD New unified subsea pressure sensor
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD ROS plugin base classes for ModelPlugin and SensorPlugin
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* MV Gazebo Protobuf messages to new package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* MV Sensor meshes to new package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Unique sensor plugin package
  Generation of Gazebo messages can be enabled, ROS messages
  will always be generated by all sensor plugins.
  This will diminish the complexity of the sensor plugin structure.
  Option to use the static TF frame "world_ned" instead of the
  default "world" frame.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Marcusso Manhaes Musa Morena (CR/AEI), Musa Morena Marcusso Manhaes
