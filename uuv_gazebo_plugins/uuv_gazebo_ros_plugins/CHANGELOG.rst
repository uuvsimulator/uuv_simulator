^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_gazebo_ros_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#223 <https://github.com/uuvsimulator/uuv_simulator/issues/223>`_ from uuvsimulator/feature/auv_control_allocation
  Feature/auv control allocation
* CHANGE Version
* Merge pull request `#216 <https://github.com/uuvsimulator/uuv_simulator/issues/216>`_ from uuvsimulator/fix/added_mass_offset_service
  FIX Service name for added-mass offset function
* FIX Service name for added-mass offset function
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#212 <https://github.com/uuvsimulator/uuv_simulator/issues/212>`_ from uuvsimulator/fix/gazebo_9_compatibility
  Fix/gazebo 9 compatibility
* ADD Update the calls for the new Gazebo 9 API
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#193 <https://github.com/uuvsimulator/uuv_simulator/issues/193>`_ from uuvsimulator/feature/control_wrt_ned_frame
  Feature/control wrt ned frame
* ADD Dependency to tf2_ros
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD TF broadcaster for base_link_ned
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Generation of base_link_ned frame
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#186 <https://github.com/uuvsimulator/uuv_simulator/issues/186>`_ from uuvsimulator/feature/param_scaling_and_offset
  Feature/param scaling and offset
* ADD Service set/get callback functions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#182 <https://github.com/uuvsimulator/uuv_simulator/issues/182>`_ from uuvsimulator/fix/gm_process_random_seed
  Fix/gm process random seed
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#173 <https://github.com/uuvsimulator/uuv_simulator/issues/173>`_ from uuvsimulator/fix/travis_build_tests
  Fix/travis build tests
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#169 <https://github.com/uuvsimulator/uuv_simulator/issues/169>`_ from uuvsimulator/fix/update_battery_topic_exception
  FIX Optionally subscribe to device state topic if it is not given in …
* FIX Optionally subscribe to device state topic if it is not given in URDF
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#149 <https://github.com/uuvsimulator/uuv_simulator/issues/149>`_ from uuvsimulator/fix/thruster_gain_auv
  Fix/thruster gain auv
* FIX Typo in fin macro snippet
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#138 <https://github.com/uuvsimulator/uuv_simulator/issues/138>`_ from uuvsimulator/feature/js_publisher_moving_joints
  Feature/js publisher moving joints
* ADD Joint state publisher snippet to misc.xacro
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Retrieve automatically the moving joints
  Since the fixed joints are still not parsed correctly by the
  URDF parser, test the joints with zero upper and lower limits
  to consider them as fixed joints by default.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#137 <https://github.com/uuvsimulator/uuv_simulator/issues/137>`_ from uuvsimulator/feature/gazebo_ros_plugins_urdf_snippets
  ADD URDF snippets to Gazebo ROS plugins
* Merge pull request `#136 <https://github.com/uuvsimulator/uuv_simulator/issues/136>`_ from uuvsimulator/feature/battery_plugins
  Feature/battery plugins
* Merge pull request `#135 <https://github.com/uuvsimulator/uuv_simulator/issues/135>`_ from uuvsimulator/feature/default_topics_actuators
  Feature/default topics actuators
* ADD URDF snippets to Gazebo ROS plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Custom battery consumer ROS plugin
  It can subscribe to a device state topic to turn the consumer on and
  off.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD ROS plugin for linear battery model
  Model plugin is inherited from Gazebo's linear battery plugin.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Add list of plugins to append new modules
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Input fin ID and default topic tags
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Move configuration output to ROS plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#104 <https://github.com/uuvsimulator/uuv_simulator/issues/104>`_ from uuvsimulator/dev/travis_integration
  Dev/travis integration
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#100 <https://github.com/uuvsimulator/uuv_simulator/issues/100>`_ from uuvsimulator/fix/target_dependencies
  Fix/target dependencies
* FIX Dependencies to catkin targets to avoid warnings.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#70 <https://github.com/uuvsimulator/uuv_simulator/issues/70>`_ from uuvsimulator/dev/integration_gazebo_gps
  Dev/integration gazebo gps
* ADD Publish topic with flag is_submerged for every vehicle.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#65 <https://github.com/uuvsimulator/uuv_simulator/issues/65>`_ from uuvsimulator/fix/catkin_tools_config
  Fix/catkin tools config
* CHANGE Package configuration for catkin tools.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#62 <https://github.com/uuvsimulator/uuv_simulator/issues/62>`_ from uuvsimulator/dev/get_hydro_model_service
  Dev/get hydro model service
* ADD Callback to service to return the model parameter of all links that have a Fossen model running.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge branch 'master' of https://github.com/uuvsimulator/uuv_simulator
* Merge pull request `#34 <https://github.com/uuvsimulator/uuv_simulator/issues/34>`_ from uuvsimulator/dev/thruster_state_publishing
  Dev/thruster state publishing
* CHANGE Use thruster plugin topic prefix to generate topic and service names.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Publishing the thruster states as ROS topics.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#29 <https://github.com/uuvsimulator/uuv_simulator/issues/29>`_ from uuvsimulator/dev/3d_current
  Dev/3d current
* Publishing topic for the flag for using local or global current velocity information.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adding implementation of the function to publish current velocity marker.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adding function to publish the current velocity marker.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adapting code to the coding style guide.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adding licensing information.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#22 <https://github.com/uuvsimulator/uuv_simulator/issues/22>`_ from uuvsimulator/Fix/wrong_angular_accel
  workaround for Gazebo reporting wrong angular accelerations:
* Added AccelerationsTestPlugin to show problem with
  Gazebo's angular accelerations. (Reported angular
  acceleration differs significantly from the one
  obtained by numerical differentiation).
  Signed-off-by: Sebastian Scherer <Sebastian.Scherer2@de.bosch.com>
* Merge pull request `#19 <https://github.com/uuvsimulator/uuv_simulator/issues/19>`_ from uuvsimulator/fix/thruster_efficiency_msg
  Adding a message to the thruster efficiency method callback functions.
* Adding a message to the thruster efficiency method callback functions.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#18 <https://github.com/uuvsimulator/uuv_simulator/issues/18>`_ from uuvsimulator/dev/local_current_vel
  Adding the option to read the current velocity under the vehicle name…
* Merge pull request `#17 <https://github.com/uuvsimulator/uuv_simulator/issues/17>`_ from uuvsimulator/dev/fin_operation
  Dev/fin operation
* Adding the option to read the current velocity under the vehicle namespace. Useful when the current velocity is read and interpolated from a file.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Correcting the update of the publish time stamp. Publishing the wrench messages.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adding a ROS topic for each fin force vector
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#1 <https://github.com/uuvsimulator/uuv_simulator/issues/1>`_ from uuvsimulator/devel
  Initial public release
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer, Sebastian Scherer (CR/AEI), lurido, sebastianscherer
