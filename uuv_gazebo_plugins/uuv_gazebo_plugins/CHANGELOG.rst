^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.13 (2019-08-12)
-------------------

0.6.12 (2019-05-23)
-------------------

0.6.11 (2019-03-21)
-------------------

0.6.10 (2019-02-28)
-------------------
* Fix dynamic linking for test build
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Replace boost::shared_ptr for std::shared_ptr
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Add boost library dependencies to test build
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix errors from catkin_lint
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.6.9 (2019-02-26)
------------------

0.6.8 (2019-02-14)
------------------
* Fix bug where thruster axis is determined incorrectly if it rotates about the negative axis
  Signed-off-by: Jordan Lack <jlack1987@gmail.com>
* Contributors: Jordan Lack

0.6.7 (2019-02-13)
------------------

0.6.6 (2019-02-12)
------------------

0.6.5 (2019-02-07)
------------------
* Fix gazebo_dev dependency
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.4 (2019-02-03)
------------------
* FIX Dependency errors for ROS Buildfarm
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.3 (2018-12-13)
------------------
* FIX Access to joint axis and pose for Gazebo 9 API
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Use lowercase strings for e-mail
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Removed Unicode characters U+201C and U+2013
  Signed-off-by: Henrique Baqueiro <hbaqueiro@gmail.com>
* Update thruster manager python to parse joint axes from urdf string via robot_description
  Update gazebo ThrusterPlugin to get the thruster joint axis from the Joint API instead of hard coding it to be the x-axis
  Signed-off-by: Jordan Lack <jlack1987@gmail.com>
* Contributors: Henrique Baqueiro, Jordan Lack, Musa Morena Marcusso Manhães

0.6.2 (2018-12-03)
------------------
* ADD rosunit to test dependency
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Compilation configuration
  * Move add_livrary for Gazebo Protobuf messages after catkin_package (fix catkin lint output)
  * Add `uuv\_` prefix to all plugins generated
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
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
* RM Installation of test folder
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Changed submergedHeight calculation to be more precise
  Signed-off-by: Guy Stoppi <gstoppi@clearpathrobotics.com>
* Applied buoyancy force in the world frame to surface vessels
  Signed-off-by: Guy Stoppi <gstoppi@clearpathrobotics.com>
* Contributors: Guy Stoppi, Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães

0.5.7 (2018-07-06)
------------------
* ADD Service handlers to retrieve parameters from the internal models
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.6 (2018-07-06)
------------------

0.5.5 (2018-07-05)
------------------
* UPDATE Package description
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Setting nonlinear coefficients to the correct matrix
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
* ADD CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.1 (2018-07-03)
------------------
* CHANGE Bump version to 0.5.2
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Output for the received lift and drag constants
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Receive the expected submerged height instead of water level area
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Set buoyancy force to a constant vector once it stabilizes
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Store fin joint angle to be used to update the output topic
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Version
* FIX Ensure volume is not negative when offset is provided
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Update the calls for the new Gazebo 9 API
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Generation of base_link_ned frame
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Estimation of volume for buoyancy
  For Collada geometries the Gazebo API cannot compute the volume.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Compute volume from collision geometries only by Gazebo version >= 7
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Get volume from collision geometry
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Hyd. model scaling and offset parameters
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Volume scaling and offset parameters
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Add list of plugins to append new modules
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Input fin ID and default topic tags
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Move configuration output to ROS plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Read parameters for the metacenter
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Parameters to simulate simple surface vessels
  The buoyancy module can now receive parameters such as the the
  metacenter of the vessel and generate the buoyancy force and torque
  according to the simple boxed-shaped surface vessel model.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Test to ensure that accelerations are valid during the simulation.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Generation of cylinder hydrodynamic parameters.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Optional input argument to limit the output thrust force.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Publish topic with flag is_submerged for every vehicle.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* install message library
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* include & export path with generated message files
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* CHANGE Package configuration for catkin tools.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Return flag whether the object is neutrally buoyant.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD String header.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Get method for model parameters.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Initialization of wrong vector for lin. damping coefficients (proportional to forward speed).
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* RM Old computed accelerations.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Modifying the parameter plotting function call.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Linear damping proportional to the forward speed.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Implementation of converters to and from the NED convention.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Thruster topic prefix as attribute of the Gazebo's plugin class.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding thruster ID parameter to the thruster plugin.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adapting code to the coding style guide.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding function to publish the current velocity RViz marker.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adapting code to the coding style.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* fix not initialized variable, prevent division by zero
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* workaround for Gazebo reporting wrong angular accelerations:
  use numerical differentiation of velocities
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Adding the option to read the current velocity under the vehicle namespace. Useful when the current velocity is read and interpolated from a file.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Minor corrections to comply with Gazebo's coding style
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Calculating the relative velocity in the fin plugin.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Correcting the update of the publish time stamp. Publishing the wrench messages.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding a ROS topic for each fin force vector
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Consider cross term when computing acceleration wrt body frame.
  Avoid Gazebo's "RelativeAccel" functions due to their ambiguous meaning.
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Asserting Gazebo coding style
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Name of variable changed.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Setting gravitational acceleration from the physics engine to the buoyant object.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* fix several files not being installed (can now source install/setup.bash)
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Fixing the sign for the added-mass Coriolis coefficients.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer, Sebastian Scherer (CR/AEI)
