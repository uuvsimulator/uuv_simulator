^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_gazebo_ros_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.13 (2019-08-12)
-------------------

0.6.11 (2019-03-21)
-------------------

0.6.12 (2019-05-23)
-------------------

0.6.10 (2019-02-28)
-------------------
* Fix errors from catkin_lint
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.6.9 (2019-02-26)
------------------
* Replace all gazebo dependencies for gazebo_dev
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.6.8 (2019-02-14)
------------------
* Fix paths to launch files after refactor
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

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
* Contributors: Musa Morena Marcusso Manhães

0.6.3 (2018-12-13)
------------------
* CHANGE Use lowercase strings for e-mail
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Box inertia macro to thruster test URDF file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Start empty test world for all tests
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Empty world file for uuv_gazebo_ros_plugins tests
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add uuv_descriptions test depend to uuv_gazebo_ros_plugins
  Signed-off-by: Will Baker <willcbaker@gmail.com>
* Correct-ize wrench vector from ThrusterROSPlugin
  Signed-off-by: Will Baker <wbaker@houstonmechatronics.com>
* Contributors: Musa Morena Marcusso Manhães, Will Baker

0.6.2 (2018-12-03)
------------------
* ADD uuv\_ prefix to plugin names
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Circular dependency for uuv_descriptions
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Compilation configuration
  * Add `uuv\_`prefix to all plugins
  * Add missing REQUIRED option to test dependencies (fix catkin lint error)
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.1 (2018-08-03)
------------------
* ADD xacro dependency for ROS tests
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.6.0 (2018-07-31)
------------------

0.5.13 (2018-07-24)
-------------------

0.5.12 (2018-07-23)
-------------------

0.5.11 (2018-07-21)
-------------------
* ADD ROS tests to CMakeLists
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX ROS test files
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.10 (2018-07-10)
-------------------

0.5.9 (2018-07-09)
------------------

0.5.8 (2018-07-07)
------------------

0.5.7 (2018-07-06)
------------------
* ADD Service handlers to get parameters from fin and thruster plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Services and messages to retrieve parameters from UUV Gazebo plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.6 (2018-07-06)
------------------

0.5.5 (2018-07-05)
------------------
* ADD ROS tests for verification of default Fossen and sphere vehicle models
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
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
* 0.5.2
* ADD CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.1 (2018-07-03)
------------------
* CHANGE Bump version to 0.5.2
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Version
* FIX Service name for added-mass offset function
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Update the calls for the new Gazebo 9 API
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Dependency to tf2_ros
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD TF broadcaster for base_link_ned
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Generation of base_link_ned frame
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Service set/get callback functions
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Optionally subscribe to device state topic if it is not given in URDF
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Typo in fin macro snippet
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Joint state publisher snippet to misc.xacro
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Retrieve automatically the moving joints
  Since the fixed joints are still not parsed correctly by the
  URDF parser, test the joints with zero upper and lower limits
  to consider them as fixed joints by default.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD URDF snippets to Gazebo ROS plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Custom battery consumer ROS plugin
  It can subscribe to a device state topic to turn the consumer on and
  off.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD ROS plugin for linear battery model
  Model plugin is inherited from Gazebo's linear battery plugin.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Add list of plugins to append new modules
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Input fin ID and default topic tags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Move configuration output to ROS plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Dependencies to catkin targets to avoid warnings.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Publish topic with flag is_submerged for every vehicle.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Package configuration for catkin tools.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Callback to service to return the model parameter of all links that have a Fossen model running.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Use thruster plugin topic prefix to generate topic and service names.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Publishing the thruster states as ROS topics.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Publishing topic for the flag for using local or global current velocity information.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding implementation of the function to publish current velocity marker.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding function to publish the current velocity marker.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adapting code to the coding style guide.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding licensing information.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Added AccelerationsTestPlugin to show problem with
  Gazebo's angular accelerations. (Reported angular
  acceleration differs significantly from the one
  obtained by numerical differentiation).
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Adding a message to the thruster efficiency method callback functions.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding the option to read the current velocity under the vehicle namespace. Useful when the current velocity is read and interpolated from a file.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Correcting the update of the publish time stamp. Publishing the wrench messages.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding a ROS topic for each fin force vector
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer, Sebastian Scherer (CR/AEI)
