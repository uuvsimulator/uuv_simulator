^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_trajectory_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* FIX Heading offset reference for 2 waypoints
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <Musa.Marcusso@de.bosch.com>
* CHANGE Default parameters on interpolators
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <Musa.Marcusso@de.bosch.com>
* Merge pull request `#244 <https://github.com/uuvsimulator/uuv_simulator/issues/244>`_ from uuvsimulator/fix/auv_controller_logic
  Fix/auv controller logic
* ADD Velocity control terms
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Using the user input for idle circle radius
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Thruster configuration parameters to controller node
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#240 <https://github.com/uuvsimulator/uuv_simulator/issues/240>`_ from uuvsimulator/fix/dubins_max_pitch
  FIX Computation of max. Z step in Dubins algorithm
* FIX Computation of max. Z step in Dubins algorithm
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#238 <https://github.com/uuvsimulator/uuv_simulator/issues/238>`_ from uuvsimulator/feature/time_to_target
  ADD Publish estimated time to target
* ADD Publish estimated time to target
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#236 <https://github.com/uuvsimulator/uuv_simulator/issues/236>`_ from uuvsimulator/fix/control_header
  ADD Header to file
* ADD Header to file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#235 <https://github.com/uuvsimulator/uuv_simulator/issues/235>`_ from uuvsimulator/feature/auv_geometric_control
  Feature/auv geometric control
* FIX Test for proximity of the final waypoint for Dubins path
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Initial heading of the vehicle to perform go to waypoint
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Simple geometric controller for AUVs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#234 <https://github.com/uuvsimulator/uuv_simulator/issues/234>`_ from uuvsimulator/fix/add_namespace_to_log
  ADD Robot namespace to log output
* ADD Robot namespace to log output
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#233 <https://github.com/uuvsimulator/uuv_simulator/issues/233>`_ from uuvsimulator/fix/approach_with_dubins
  FIX Turn off smooth approach if Dubins path is being used
* FIX Turn off smooth approach if Dubins path is being used
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#232 <https://github.com/uuvsimulator/uuv_simulator/issues/232>`_ from uuvsimulator/fix/timeout_until_idle
  ADD Timeout to activate idle mode
* ADD Timeout to activate idle mode
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#231 <https://github.com/uuvsimulator/uuv_simulator/issues/231>`_ from uuvsimulator/feature/idle_radius
  ADD Input to configure circle path idle radius for AUVs
* ADD Input to configure circle path idle radius for AUVs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#229 <https://github.com/uuvsimulator/uuv_simulator/issues/229>`_ from uuvsimulator/fix/clamp_max_forward_speed
  RM Clamping max. forward speed
* RM Clamping max. forward speed
* Merge pull request `#228 <https://github.com/uuvsimulator/uuv_simulator/issues/228>`_ from uuvsimulator/fix/local_planner_tf_timeout
  FIX TF timeout
* FIX TF timeout
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#227 <https://github.com/uuvsimulator/uuv_simulator/issues/227>`_ from uuvsimulator/fix/auv_traj_control
  Fix/auv traj control
* MAINT Sliding mode controller launch
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Casadi based equations for the vehicle model
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Local planners to allow look ahead distance for AUVs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Independent method to obtain sampled of reference path
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Flags to use either fins or thrusters
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#224 <https://github.com/uuvsimulator/uuv_simulator/issues/224>`_ from uuvsimulator/feature/dubins_path
  Feature/dubins path
* ADD Linear interpolator in the package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Helical path segment generator
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Use the Bezier static method to generate curve
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Set parameters method
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* RM Normalized parameter from derivative function
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD 3D Dubins path interpolator
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Option to have either thrusters or fins as outputs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Option to add the name of the interpolator to be used
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Functions to set interpolator parameters
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Information about surge speed
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Radius of acceptance and heading calculation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Function to return all interpolator options
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Add interpolator markers and fix double generation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Function to generate series of cubic Bezier segments
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#223 <https://github.com/uuvsimulator/uuv_simulator/issues/223>`_ from uuvsimulator/feature/auv_control_allocation
  Feature/auv control allocation
* CHANGE Version
* Merge pull request `#215 <https://github.com/uuvsimulator/uuv_simulator/issues/215>`_ from uuvsimulator/fix/mb_fl_orientation_reference
  Fix/mb fl orientation reference
* ADD Method convert quaternion to rot. matrix
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Origin of the orientation information for restoring force
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#213 <https://github.com/uuvsimulator/uuv_simulator/issues/213>`_ from uuvsimulator/fix/bop_panel_model
  Fix/bop panel model
* FIX Initialization of state variables
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#209 <https://github.com/uuvsimulator/uuv_simulator/issues/209>`_ from uuvsimulator/feature/pd_with_compensation
  ADD PD controller with compensation of restoring forces
* ADD PD controller with compensation of restoring forces
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#208 <https://github.com/uuvsimulator/uuv_simulator/issues/208>`_ from uuvsimulator/fix/sf_controller_node_name
  Fix/sf controller node name
* FIX NED representation of the restoring forces vector
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Node name for the cases where a parameter file is provided
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#204 <https://github.com/uuvsimulator/uuv_simulator/issues/204>`_ from uuvsimulator/fix/sf_controller_inputs
  ADD Option to add vectors as a controller parameter input
* ADD Option to add vectors as a controller parameter input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#201 <https://github.com/uuvsimulator/uuv_simulator/issues/201>`_ from uuvsimulator/feature/sf_controller
  ADD Implementation of singularity-free tracking controller
* ADD Start implementation of derivatives
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Initial orientation input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Return initial rotation for s == 0
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Initial orientation at start of interpolation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Read time vector to trajectory generator and initial orientation input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* RM Odometry debug output
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Read vehicle orientation when starting trajectory
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Orientation error computation to SF controller
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Implementation of singularity-free tracking controller
* Merge pull request `#196 <https://github.com/uuvsimulator/uuv_simulator/issues/196>`_ from uuvsimulator/fix/update_dp_tutorial
  Fix/update dp tutorial
* ADD Mutex object to control access to the waypoint list
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Error message by message creation callback
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Block to handle exception upon receiving waypoints
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#193 <https://github.com/uuvsimulator/uuv_simulator/issues/193>`_ from uuvsimulator/feature/control_wrt_ned_frame
  Feature/control wrt ned frame
* ADD Inertial frame_id to waypoints and waypoint sets
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* RM Old waypoint classes
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Trajectory generation in both world and world_ned frame
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Consider odometry for both world and world_ned frames
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Option to use either world or world_end frame for local planner
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Subscribe to input_stamped from thruster_manager
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Log file label for each controller script
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Input for use_ned_frame and subscribe to input_stamped in thruster manager
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#182 <https://github.com/uuvsimulator/uuv_simulator/issues/182>`_ from uuvsimulator/fix/gm_process_random_seed
  Fix/gm process random seed
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#173 <https://github.com/uuvsimulator/uuv_simulator/issues/173>`_ from uuvsimulator/fix/travis_build_tests
  Fix/travis build tests
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Importing uuv_waypoints in unit tests
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#170 <https://github.com/uuvsimulator/uuv_simulator/issues/170>`_ from uuvsimulator/feature/new_structure_dp_controllers
  Feature/new structure dp controllers
* ADD Angle saturation input
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Import path to the new uuv_waypoints package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Separate Python package for waypoints package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#161 <https://github.com/uuvsimulator/uuv_simulator/issues/161>`_ from uuvsimulator/fix/trajectory_control_test_path
  MV Unit tests for the trajectory control package
* MV Unit tests for the trajectory control package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#131 <https://github.com/uuvsimulator/uuv_simulator/issues/131>`_ from uuvsimulator/feature/mb_fl_controller
  ADD Model-based feedback linearization controller
* Merge pull request `#130 <https://github.com/uuvsimulator/uuv_simulator/issues/130>`_ from uuvsimulator/fix/nmb_sm_sliding_surface
  FIX Sliding surface with saturation function
* ADD Model-based feedback linearization controller
  Controller mostly targeted for thruster actuated robot models.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Sliding surface with saturation function
  To avoid extreme control efforts from being generated, add an
  option to use a saturation function that will fix the chattering
  problem with the non-model-based sliding mode controller.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#118 <https://github.com/uuvsimulator/uuv_simulator/issues/118>`_ from uuvsimulator/dev/simple_auv_tracking_controller
  Simple AUV tracking controller and PID for underactuated vehicles
* FIX Matrix dimension errors
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD PID controller for underactuated vehicles
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Argument to generate trajectories in 6 DoF
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD gui_on flag to all controller launch files
  gui_on flag will toggle publication of trajectory and waypoint visual
  markers
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE CMakeLists to install new controller script
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Simple AUV P-controller
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#116 <https://github.com/uuvsimulator/uuv_simulator/issues/116>`_ from uuvsimulator/fix/controller_init
  ADD Flag to check controller initialization
* ADD Flag to check controller initialization
  The controller could break in case the timer set in the
  base class started the update before all parameters were
  properly initialized.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#104 <https://github.com/uuvsimulator/uuv_simulator/issues/104>`_ from uuvsimulator/dev/travis_integration
  Dev/travis integration
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#94 <https://github.com/uuvsimulator/uuv_simulator/issues/94>`_ from uuvsimulator/dev/traj_stamed_poses
  ADD Read flag to use stamped poses from parameter server.
* ADD Read flag to use stamped poses from parameter server.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#91 <https://github.com/uuvsimulator/uuv_simulator/issues/91>`_ from uuvsimulator/dev/integration_uuv_teleop
  Dev/integration uuv teleop
* ADD Restriction to the teleop pose reference regarding the sea surface
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Set methods for the position vector
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Teleop method reading reference input from the joystick to the DP controller local planner
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Option to generate reference with stamped poses only
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD List to map segments to waypoints to trace the vehicle.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#81 <https://github.com/uuvsimulator/uuv_simulator/issues/81>`_ from uuvsimulator/dev/preparation_mangalia
  Dev/preparation mangalia
* ADD Test for waypoint above sea surface (Gazebo's ENU frame)
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* RM Deprecated computation of straight lines, now using lipb
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Method to return the current damping matrix
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD start_station_keeping method
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Test the max. forward speed input for the go to waypoint command.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Output of NaN time of trajectory point message.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Hold vehicle if trajectory is finished
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX None as reference in DP controller.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Local planner for straight line paths.
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Merge pull request `#65 <https://github.com/uuvsimulator/uuv_simulator/issues/65>`_ from uuvsimulator/fix/catkin_tools_config
  Fix/catkin tools config
* ADD launch folder to be installed.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#61 <https://github.com/uuvsimulator/uuv_simulator/issues/61>`_ from uuvsimulator/fix/controller_vehicle_interface
  Fix/controller vehicle interface
* FIX Conversion to/from SNAME convention in the local vehicle model.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#55 <https://github.com/uuvsimulator/uuv_simulator/issues/55>`_ from uuvsimulator/fix/dp_evaluation_logging
  Fix/dp evaluation logging
* ADD Alterations from fix/dp_evaluation_logging
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Logging to the controllers.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#54 <https://github.com/uuvsimulator/uuv_simulator/issues/54>`_ from uuvsimulator/fix/reset_trajectory_generator
  Fix/reset trajectory generator
* FIX Reset the waypoint interpolator between service calls
  FIX Missing links for the trajectory interpolation.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#51 <https://github.com/uuvsimulator/uuv_simulator/issues/51>`_ from uuvsimulator/fix/start_offset_trajectory
  Fix/start offset trajectory
* FIX Return trajectory's start pose reference if a start time offset was given.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Use trajectory duration instead of max. time.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* FIX Integrator signal in controller abstract class.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Instead of trajectory max. time, use duration as offset wrt start time.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* FIX Sign of the integrator.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge pull request `#44 <https://github.com/uuvsimulator/uuv_simulator/issues/44>`_ from uuvsimulator/fix/trajectory_termination
  Fix/trajectory termination
* FIX Grammar error in comment.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Min. radius for polynomial blend according to the neighboring line segments.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Bibliographic reference for the linear interpolation with polynomial blends.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Test if the Bezier curve order provided is valid.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Template files to build a new thruster actuated underwater vehicle.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* FIX Correcting type of flag variables for the MB SM controller.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* FIX Missing colon.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Using cubic interpolation now only for helical and circular trajectories.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Bezier curves and linear segments to the path_generator package.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Adaptation of the waypoint interpolator to the new interpolator implementations.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE Cubic interpolator to use the Bezier curves class instead of the scipy implementation.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Class to compute linear interpolation of waypoints with polynomial blends.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Class for computation of linear segments used by the LIPB interpolator.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Class for computation of 3D Bezier curves (order 3, 4 and 5).
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Demonstration of cubic and linear interpolation with polynomial blends.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* CHANGE NMB SM parameter setting.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* FIX Computation from max. time from the interpolated waypoint path.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* FIX Test to set the finishing flag of a trajectory from an waypoint interpolated path.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Merge branch 'master' into dev/cart_control_vel_reference
* Merge branch 'master' of https://github.com/uuvsimulator/uuv_simulator
* Merge pull request `#41 <https://github.com/uuvsimulator/uuv_simulator/issues/41>`_ from uuvsimulator/dev/rov_trajectory_control
  Dev/rov trajectory control
* ADD Link to the SMAC repository.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* FIX Removing comment characters in wrong enconding
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Test units for some trajectory generator
  modules.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Demo script for the waypoint interpolator.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Controller and RexROV vehicle model parameter
  files.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Trajectory controller ROS nodes and launch
  files.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Setup file for the trajectory control Python
  modules
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD DP controller abstract classes.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD Trajectory generation Python module.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* ADD New package with trajectory controllers.
  Signed-off-by: Musa Morena Marcusso Manhães <Musa.Marcusso@de.bosch.com>
* Contributors: Marcusso Manhaes Musa Morena (CR/AEI), Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Sebastian Scherer, lurido, sebastianscherer
