^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#244 <https://github.com/uuvsimulator/uuv_simulator/issues/244>`_ from uuvsimulator/fix/auv_controller_logic
  Fix/auv controller logic
* ADD Output for the received lift and drag constants
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#241 <https://github.com/uuvsimulator/uuv_simulator/issues/241>`_ from uuvsimulator/fix/surface_vessel_buoyancy
  Fix/surface vessel buoyancy
* FIX Receive the expected submerged height instead of water level area
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Set buoyancy force to a constant vector once it stabilizes
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#223 <https://github.com/uuvsimulator/uuv_simulator/issues/223>`_ from uuvsimulator/feature/auv_control_allocation
  Feature/auv control allocation
* FIX Store fin joint angle to be used to update the output topic
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Version
* Merge pull request `#217 <https://github.com/uuvsimulator/uuv_simulator/issues/217>`_ from uuvsimulator/fix/volume_offset
  FIX Ensure volume is not negative when offset is provided
* FIX Ensure volume is not negative when offset is provided
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#212 <https://github.com/uuvsimulator/uuv_simulator/issues/212>`_ from uuvsimulator/fix/gazebo_9_compatibility
  Fix/gazebo 9 compatibility
* ADD Update the calls for the new Gazebo 9 API
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#193 <https://github.com/uuvsimulator/uuv_simulator/issues/193>`_ from uuvsimulator/feature/control_wrt_ned_frame
  Feature/control wrt ned frame
* ADD Generation of base_link_ned frame
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#190 <https://github.com/uuvsimulator/uuv_simulator/issues/190>`_ from uuvsimulator/feature/sensor_with_ned_static_ref
  Feature/sensor with ned static ref
* RM Estimation of volume for buoyancy
  For Collada geometries the Gazebo API cannot compute the volume.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Compute volume from collision geometries only by Gazebo version >= 7
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* RM Get volume from collision geometry
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#186 <https://github.com/uuvsimulator/uuv_simulator/issues/186>`_ from uuvsimulator/feature/param_scaling_and_offset
  Feature/param scaling and offset
* ADD Hyd. model scaling and offset parameters
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Volume scaling and offset parameters
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#182 <https://github.com/uuvsimulator/uuv_simulator/issues/182>`_ from uuvsimulator/fix/gm_process_random_seed
  Fix/gm process random seed
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#173 <https://github.com/uuvsimulator/uuv_simulator/issues/173>`_ from uuvsimulator/fix/travis_build_tests
  Fix/travis build tests
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#135 <https://github.com/uuvsimulator/uuv_simulator/issues/135>`_ from uuvsimulator/feature/default_topics_actuators
  Feature/default topics actuators
* CHANGE Add list of plugins to append new modules
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Input fin ID and default topic tags
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Move configuration output to ROS plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#114 <https://github.com/uuvsimulator/uuv_simulator/issues/114>`_ from uuvsimulator/dev/simple_boxed_surface_vessel
  Dev/simple boxed surface vessel
* Merge pull request `#113 <https://github.com/uuvsimulator/uuv_simulator/issues/113>`_ from uuvsimulator/dev/thruster_limits
  ADD Optional input argument to limit the output thrust force.
* ADD Read parameters for the metacenter
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Parameters to simulate simple surface vessels
  The buoyancy module can now receive parameters such as the the
  metacenter of the vessel and generate the buoyancy force and torque
  according to the simple boxed-shaped surface vessel model.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#104 <https://github.com/uuvsimulator/uuv_simulator/issues/104>`_ from uuvsimulator/dev/travis_integration
  Dev/travis integration
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#103 <https://github.com/uuvsimulator/uuv_simulator/issues/103>`_ from uuvsimulator/dev/manipulator_eval
  Dev/manipulator eval
* ADD Test to ensure that accelerations are valid during the simulation.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Generation of cylinder hydrodynamic parameters.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Optional input argument to limit the output thrust force.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#70 <https://github.com/uuvsimulator/uuv_simulator/issues/70>`_ from uuvsimulator/dev/integration_gazebo_gps
  Dev/integration gazebo gps
* ADD Publish topic with flag is_submerged for every vehicle.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#65 <https://github.com/uuvsimulator/uuv_simulator/issues/65>`_ from uuvsimulator/fix/catkin_tools_config
  Fix/catkin tools config
* install message library
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* include & export path with generated message files
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* CHANGE Package configuration for catkin tools.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#62 <https://github.com/uuvsimulator/uuv_simulator/issues/62>`_ from uuvsimulator/dev/get_hydro_model_service
  Dev/get hydro model service
* ADD Return flag whether the object is neutrally buoyant.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD String header.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Get method for model parameters.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge branch 'master' of https://github.com/uuvsimulator/uuv_simulator
* Merge pull request `#40 <https://github.com/uuvsimulator/uuv_simulator/issues/40>`_ from uuvsimulator/fix/lin_damping_forward_speed
  FIX Initialization of wrong vector for lin. damping coefficients (pro…
* FIX Initialization of wrong vector for lin. damping coefficients (proportional to forward speed).
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#38 <https://github.com/uuvsimulator/uuv_simulator/issues/38>`_ from uuvsimulator/dev/forward_speed_damping
  Dev/forward speed damping
* RM Old computed accelerations.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Modifying the parameter plotting function call.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Linear damping proportional to the forward speed.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#37 <https://github.com/uuvsimulator/uuv_simulator/issues/37>`_ from uuvsimulator/dev/ned_convention
  Dev/ned convention
* ADD Implementation of converters to and from the NED convention.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge branch 'master' of https://github.com/uuvsimulator/uuv_simulator
* Merge pull request `#34 <https://github.com/uuvsimulator/uuv_simulator/issues/34>`_ from uuvsimulator/dev/thruster_state_publishing
  Dev/thruster state publishing
* ADD Thruster topic prefix as attribute of the Gazebo's plugin class.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adding thruster ID parameter to the thruster plugin.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#29 <https://github.com/uuvsimulator/uuv_simulator/issues/29>`_ from uuvsimulator/dev/3d_current
  Dev/3d current
* Adapting code to the coding style guide.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adding function to publish the current velocity RViz marker.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adapting code to the coding style.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#23 <https://github.com/uuvsimulator/uuv_simulator/issues/23>`_ from uuvsimulator/fix/numeric_accel
  fix not initialized variable, prevent division by zero
* fix not initialized variable, prevent division by zero
  Signed-off-by: Sebastian Scherer <Sebastian.Scherer2@de.bosch.com>
* Merge pull request `#22 <https://github.com/uuvsimulator/uuv_simulator/issues/22>`_ from uuvsimulator/Fix/wrong_angular_accel
  workaround for Gazebo reporting wrong angular accelerations:
* workaround for Gazebo reporting wrong angular accelerations:
  use numerical differentiation of velocities
  Signed-off-by: Sebastian Scherer <Sebastian.Scherer2@de.bosch.com>
* Merge pull request `#18 <https://github.com/uuvsimulator/uuv_simulator/issues/18>`_ from uuvsimulator/dev/local_current_vel
  Adding the option to read the current velocity under the vehicle name…
* Merge pull request `#17 <https://github.com/uuvsimulator/uuv_simulator/issues/17>`_ from uuvsimulator/dev/fin_operation
  Dev/fin operation
* Adding the option to read the current velocity under the vehicle namespace. Useful when the current velocity is read and interpolated from a file.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Minor corrections to comply with Gazebo's coding style
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Calculating the relative velocity in the fin plugin.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Correcting the update of the publish time stamp. Publishing the wrench messages.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Adding a ROS topic for each fin force vector
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#12 <https://github.com/uuvsimulator/uuv_simulator/issues/12>`_ from uuvsimulator/fix/neutral_buoyancy
  Fix/neutral buoyancy
* Merge pull request `#13 <https://github.com/uuvsimulator/uuv_simulator/issues/13>`_ from uuvsimulator/fix/accel_crossterms
  Consider cross term when computing acceleration wrt body frame.
* Consider cross term when computing acceleration wrt body frame.
  Avoid Gazebo's "RelativeAccel" functions due to their ambiguous meaning.
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Asserting Gazebo coding style
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Name of variable changed.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Setting gravitational acceleration from the physics engine to the buoyant object.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Merge pull request `#9 <https://github.com/uuvsimulator/uuv_simulator/issues/9>`_ from uuvsimulator/fix/catkin_install
  fix several files not being installed (can now source install/setup.bash)
* fix several files not being installed (can now source install/setup.bash)
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Merge pull request `#7 <https://github.com/uuvsimulator/uuv_simulator/issues/7>`_ from uuvsimulator/fix/coriolis
  Fixing the sign for the added-mass Coriolis coefficients.
* Fixing the sign for the added-mass Coriolis coefficients.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Merge pull request `#1 <https://github.com/uuvsimulator/uuv_simulator/issues/1>`_ from uuvsimulator/devel
  Initial public release
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer, Sebastian Scherer (CR/AEI), lurido, sebastianscherer
