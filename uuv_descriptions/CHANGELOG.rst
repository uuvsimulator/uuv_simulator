^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_descriptions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.13 (2019-08-12)
-------------------

0.6.12 (2019-05-23)
-------------------
* uuv_descriptions: add xacro as a runtime dependency
  If xacro is not installed then it is not possible to spawn ROVs using upload_rexrov.launch.
* Contributors: Russ

0.6.11 (2019-03-21)
-------------------

0.6.10 (2019-02-28)
-------------------
* Fix dependency for message_to_tf node
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix name of package for uuv_message_to_tf
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix errors from catkin_lint
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Fix non-executables
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

0.6.3 (2018-12-13)
------------------
* CHANGE Use lowercase strings for e-mail
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Path to meshes folder after refactoring
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* MV World models and files to uuv_gazebo_worlds
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.6.2 (2018-12-03)
------------------
* ADD uuv\_ prefix to plugin names
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

0.5.7 (2018-07-06)
------------------

0.5.6 (2018-07-06)
------------------
* ADD Gazebo world with physics engine configured for AUV models
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães

0.5.5 (2018-07-05)
------------------
* ADD Author to the tree world model
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Merge messages from the change log
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* UPDATE CHANGELOG files
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
* UPDATE Logs
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD CHANGELOG files
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.5.1 (2018-07-03)
------------------
* CHANGE Bump version to 0.5.2
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Launch files for the RexROV using spawn_model
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD spawn_model script to CMakeLists.txt
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE spawn_model script to use geodetic coordinates
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Option to spawn vehicle using geodetic coordinates
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX World model emission properties
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Version
* FIX Corrected marker size for improved detection
  Signed-off-by: Luiz Ricardo Douat <LuizRicardo.Doaut@de.bosch.com>
* FIX Use FIDUALS-ARUCO instead of RCARS-APRILTAGS
  Signed-off-by: Luiz Ricardo Douat <LuizRicardo.Doaut@de.bosch.com>
* FIX Add missing STL geometry
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Lighting properties
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Set cast_shadows to false
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD robot_state_publisher dependency
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Replace DAE for OBJ
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Set cast_shadows to false
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Relative paths to textures and scripts for Gazebo 7.9
  Signed-off-by: Marcusso Manhaes Musa Morena (CR/AEI) <musa.marcusso@de.bosch.com>
* FIX Paths for scripts and textures for Gazebo 7.9 update
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD New marker tags for BOP Panel
  FIX Oberon7 serial_arm parameters
  Signed-off-by: Luiz Ricardo Douat <luizricardo.douat@de.bosch.com>
* FIX: Correct position of subsea panel in the world.
  Signed-off-by: Luiz Ricardo Douat <luizricardo.douat@de.bosch.com>
* ADD Simple BOP panel for manipulation scenarios
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Two worlds based on the region of the Trondheim
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Option to use NED inertial frame on launch files
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Path to sensor snippets for RexROV robot description
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Path to uuv_sensor_ros_plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX NED frame Gazebo model
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Source of sensor snippets and option for world_frame input
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Publisher for static NED frame to all worlds
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Typos and package version
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Actuator and sensor xacro files for RexROV
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Joint state publisher macros for RexROV
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* MV Joint state publisher snippets
  Now in misc.xacro from uuv_gazebo_ros_plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Plugin configuration for the new world plugins
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Ned link, transformations now computed in the plugin
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* RM Redundant definition of PI
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Package dependencies for rosdep
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Gazebo models for some ROV manipulator tools.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* add Mangalia world_model and world from SWARMs black sea demonstrations
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* add world model for static surface vessel aurora
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* FIX Joint state publisher update rate
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Fiducial world models and materials
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Joint state publisher update rate
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Name of dependency package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD RexROV configuration with noisy pose_gt
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Runtime dependency for the descriptions package.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* CHANGE Current demo launch files to include the Oberon 7 arm.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Oberon 7 control package
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD RexROV + Oberon 7 launch file.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Update rate for joint state publishers
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Two more cameras to the RexROV vehicle.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD GPS sensor to the RexROV vehicle
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Lat/Long origin to the lake and ocean waves worlds.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* ADD Lat/Long origin to the empty underwater world.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Ocean model configuration name.
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Lake model name
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* FIX Gazebo specific information in RexROV launch
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* limit rate of robot_state_publisher
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* fix path to meshes in rov_bop_panel
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* RM Debug flag from rexrov_base macro.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Reference to the RexROV parameters.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Call for the underwater object plugin for all configuration of the RexROV using the new structure.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD License information.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* MV RexROV underwater object plugin parameters to a new file.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* install missing launch file
* MV world_md
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* MV BOP panel meshes to meshes/
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* FIX Link to sand texture.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* MV World files back to uuv_descriptions.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* MV World related models and descriptions to uuv_gazebo.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* RM World and world models installation and moving to uuv_gazebo.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* MV Scenario launch files from uuv_descriptions to uuv_gazebo.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* MV worlds folder from uuv_descriptions to uuv_gazebo.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* MV Contents from world_models in uuv_description to models in uuv_gazebo
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Central materials folders with shaders and textures.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Publication of RViz markers for the empty underwater world.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Node to publish the RViz markers for each Gazebo static model.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* ADD Option to add a simulation timeout to the world launch files.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* CHANGE Add only thruster ID instead of the thruster's topics for input and output. Thruster topic prefix will be generated automatically using the ID.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Increase the angle range for the current velocity vector.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* fixed incompletely modified line (new spawn_model.py)
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* add and use modified spawn_model script
  Allow setting the initial vehicle pose from another node via rosparams
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Adding the name of the child frame to message_to_tf launch file.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* fix message_to_tf, which stopped working for me recently
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Adapting world files to the new model of 3D constant currents.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* rexrov_base: replace collision mesh with primitives
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Added AccelerationsTestPlugin to show problem with
  Gazebo's angular accelerations. (Reported angular
  acceleration differs significantly from the one
  obtained by numerical differentiation).
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* New RexROV configuration with two arms (Oberon and Oberon 4) with demo launch files.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Adding robot descriptions for the RexROV + Oberon 4 arm and demo launch files.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Setting gravitational acceleration from the physics engine to the buoyant object.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* Correcting import of xml_reflection package.
  Signed-off-by: Musa Morena Marcusso Manhães <musa.marcusso@de.bosch.com>
* fix several files not being installed (can now source install/setup.bash)
  Signed-off-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
* Setting a more realistic wave amplitude to ocean shader.
  Signed-off-by: Musa Morena Marcusso Manhães (CR/AEI) <musa.marcusso@de.bosch.com>
* rename default manipulator
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* initial commit
  Signed-off-by: Sebastian Scherer (CR/AEI) <sebastian.scherer2@de.bosch.com>
* Contributors: Luiz Ricardo Douat, Marcusso Manhaes Musa Morena (CR/AEI), Musa Morena Marcusso Manhaes, Musa Morena Marcusso Manhães, Musa Morena Marcusso Manhães (CR/AEI), Sebastian Scherer, Sebastian Scherer (CR/AEI)
