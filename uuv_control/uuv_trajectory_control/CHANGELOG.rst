^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_trajectory_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.13 (2019-08-12)
-------------------
* Fix PID controller node shebang
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add private log module
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Update license header
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add docstrings
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add docstrings and update license header
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Set conditional import for the casadi package
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add logger from package's _log module
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Update license header
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Update test configuration
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Update license headers
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding docstrings and updating license headers
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fix import local waypoint module
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Fix module docstring breakline
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add missing HelicalSegment to module
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add local logging module
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add module docstring
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add uuv_assistants dependency to reach tf_quaternion package
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Replace geometry.tf.transformations by local transformations package
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add docstrings and update license headers
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Add module docstring and update license header
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.12 (2019-05-23)
-------------------

0.6.11 (2019-03-21)
-------------------
* Fix normalization of fin angle for saturation
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.10 (2019-02-28)
-------------------
* Fix errors from catkin_lint
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix non-executables
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
* Contributors: Musa Morena Marcusso Manhães

0.6.3 (2018-12-13)
------------------
* CHANGE Use lowercase strings for e-mail
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fixed the InitWaypointSet method with dp_controller_planner.py. Added interpolator to InitWaypointSet.srv
  Signed-off-by: Jackson Shields <jacksonhshields@gmail.com>
* Contributors: Musa Morena Marcusso Manhães, Jackson Shields

0.6.2 (2018-12-03)
------------------

0.6.1 (2018-08-03)
------------------

0.6.0 (2018-07-31)
------------------

0.5.13 (2018-07-24)
-------------------
* FIX Set orientation error from quaternion vector
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX If start_time is set before interpolation begins, ignore current time
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* MV Test for quaternion signal to base class
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Test for heading condition over same XY
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Read the a user-defined start_time for generation of trajectory from waypoint list
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Replace print for ROS log
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.12 (2018-07-23)
-------------------

0.5.11 (2018-07-21)
-------------------
* ADD Test quaternions dot product for shortest path
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* MV Quaternion sign test to path_generator.py
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.10 (2018-07-10)
-------------------
* CHANGE Set casadi package as optional
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Return object for handler of init_waypoint_from_file handler
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.5.9 (2018-07-09)
------------------
* FIX Transform fin angle conversion matrix for inertial frame_id=world_ned
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Export waypoint set to file with frame_id information
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>

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
* FIX Heading offset reference for 2 waypoints
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <musa.marcusso@de.bosch.com>
* CHANGE Default parameters on interpolators
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <musa.marcusso@de.bosch.com>
* ADD Velocity control terms
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Using the user input for idle circle radius
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Thruster configuration parameters to controller node
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Computation of max. Z step in Dubins algorithm
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Publish estimated time to target
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Header to file
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Test for proximity of the final waypoint for Dubins path
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Initial heading of the vehicle to perform go to waypoint
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Simple geometric controller for AUVs
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Robot namespace to log output
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Turn off smooth approach if Dubins path is being used
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Timeout to activate idle mode
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Input to configure circle path idle radius for AUVs
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Clamping max. forward speed
* FIX TF timeout
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* MAINT Sliding mode controller launch
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Casadi based equations for the vehicle model
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Local planners to allow look ahead distance for AUVs
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Independent method to obtain sampled of reference path
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Flags to use either fins or thrusters
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Linear interpolator in the package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Helical path segment generator
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Use the Bezier static method to generate curve
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Set parameters method
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Normalized parameter from derivative function
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD 3D Dubins path interpolator
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Option to have either thrusters or fins as outputs
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Option to add the name of the interpolator to be used
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Functions to set interpolator parameters
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Information about surge speed
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Radius of acceptance and heading calculation
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Function to return all interpolator options
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Add interpolator markers and fix double generation
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Function to generate series of cubic Bezier segments
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Version
* ADD Method convert quaternion to rot. matrix
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Origin of the orientation information for restoring force
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Initialization of state variables
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD PD controller with compensation of restoring forces
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX NED representation of the restoring forces vector
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Node name for the cases where a parameter file is provided
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Option to add vectors as a controller parameter input
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Start implementation of derivatives
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Initial orientation input
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Return initial rotation for s == 0
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Initial orientation at start of interpolation
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Read time vector to trajectory generator and initial orientation input
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Odometry debug output
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Read vehicle orientation when starting trajectory
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Orientation error computation to SF controller
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Implementation of singularity-free tracking controller
* ADD Mutex object to control access to the waypoint list
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Error message by message creation callback
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Block to handle exception upon receiving waypoints
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Inertial frame_id to waypoints and waypoint sets
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Old waypoint classes
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Trajectory generation in both world and world_ned frame
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Consider odometry for both world and world_ned frames
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Option to use either world or world_end frame for local planner
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Subscribe to input_stamped from thruster_manager
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Log file label for each controller script
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Input for use_ned_frame and subscribe to input_stamped in thruster manager
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Importing uuv_waypoints in unit tests
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Angle saturation input
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Import path to the new uuv_waypoints package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Separate Python package for waypoints package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* MV Unit tests for the trajectory control package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Model-based feedback linearization controller
  Controller mostly targeted for thruster actuated robot models.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Sliding surface with saturation function
  To avoid extreme control efforts from being generated, add an
  option to use a saturation function that will fix the chattering
  problem with the non-model-based sliding mode controller.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Matrix dimension errors
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD PID controller for underactuated vehicles
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Argument to generate trajectories in 6 DoF
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD gui_on flag to all controller launch files
  gui_on flag will toggle publication of trajectory and waypoint visual
  markers
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE CMakeLists to install new controller script
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Simple AUV P-controller
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Flag to check controller initialization
  The controller could break in case the timer set in the
  base class started the update before all parameters were
  properly initialized.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Read flag to use stamped poses from parameter server.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Restriction to the teleop pose reference regarding the sea surface
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Set methods for the position vector
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Teleop method reading reference input from the joystick to the DP controller local planner
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Option to generate reference with stamped poses only
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD List to map segments to waypoints to trace the vehicle.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Test for waypoint above sea surface (Gazebo's ENU frame)
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Deprecated computation of straight lines, now using lipb
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Method to return the current damping matrix
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD start_station_keeping method
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Test the max. forward speed input for the go to waypoint command.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Output of NaN time of trajectory point message.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Hold vehicle if trajectory is finished
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX None as reference in DP controller.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Local planner for straight line paths.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD launch folder to be installed.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Conversion to/from SNAME convention in the local vehicle model.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Logging to the controllers.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Reset the waypoint interpolator between service calls
  FIX Missing links for the trajectory interpolation.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Return trajectory's start pose reference if a start time offset was given.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Use trajectory duration instead of max. time.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Integrator signal in controller abstract class.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Instead of trajectory max. time, use duration as offset wrt start time.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Sign of the integrator.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Grammar error in comment.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Min. radius for polynomial blend according to the neighboring line segments.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Bibliographic reference for the linear interpolation with polynomial blends.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Test if the Bezier curve order provided is valid.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Template files to build a new thruster actuated underwater vehicle.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Correcting type of flag variables for the MB SM controller.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Missing colon.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Using cubic interpolation now only for helical and circular trajectories.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Bezier curves and linear segments to the path_generator package.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Adaptation of the waypoint interpolator to the new interpolator implementations.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Cubic interpolator to use the Bezier curves class instead of the scipy implementation.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Class to compute linear interpolation of waypoints with polynomial blends.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Class for computation of linear segments used by the LIPB interpolator.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Class for computation of 3D Bezier curves (order 3, 4 and 5).
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Demonstration of cubic and linear interpolation with polynomial blends.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE NMB SM parameter setting.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Computation from max. time from the interpolated waypoint path.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Test to set the finishing flag of a trajectory from an waypoint interpolated path.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Link to the SMAC repository.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Removing comment characters in wrong enconding
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Test units for some trajectory generator
  modules.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Demo script for the waypoint interpolator.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Controller and RexROV vehicle model parameter
  files.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Trajectory controller ROS nodes and launch
  files.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Setup file for the trajectory control Python
  modules
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD DP controller abstract classes.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Trajectory generation Python module.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD New package with trajectory controllers.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Contributors: Marcusso Manhaes Musa Morena (CR/AEI), Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães
