.. _disturbances:

Applying disturbances during the simulation
===========================================

One of the steps to evaluate the vehicle's performance using a control strategy is to check how it would behave under different kinds of disturbances.
One tool that can be used is an option in Gazebo through the GUI or using a ROS service to apply forces and torque to a link in the scenario. Additionally,
it is possible to use the services provided by this package's underwater world Gazebo plugin to set a constant current velocity to the vehicles.

To make this type of tests easier, some scripts are available in the package uuv_control_utils to schedule disturbances to happen during the simulation,
making it easier to reproduce a scenario where the disturbances have to be active only at a certain simulation time or have a limited duration. The
**uuv_control_utils** package provides scripts to apply the disturbances at specific times and also launch files that can be built in an use-case scenario.
The files mentioned below are available at the **uuv_tutorial_disturbances**.

Thruster failure
----------------

One interesting use-case to be tested is to see how the vehicle would behave in case one or more thrusters stops working. The thruster plugin provides a
service call to set the thruster state to **ON** or **OFF**. Each thruster unit generates the service names automatically as follows ::

  /<model_name>/thrusters/<thruster_id>/set_thruster_state
  /<model_name>/thrusters/<thruster_id>/get_thruster_state

The RexROV vehicle, for instance, provides the set thruster state service for thruster #2 as ::

  /rexrov/thrusters/2/set_thruster_state.

To use the service call, a few options are presented below.

Setting thruster state through ROS service call
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When you start the simulation with an thruster-actuated underwater vehicle, the thruster services will be made available by the thruster unit plugins.
So, if you start, for example, the PID controller demo launch file as ::

  roslaunch uuv_gazebo start_pid_demo.launch

you can set the state of one of the thrusters to **OFF** using the following ::

  rosservice call /rexrov/thrusters/2/set_thruster_state "'on': false"

You can check the state of the thruster unit by calling ::

  rosservice call /rexrov/thrusters/2/get_thruster_state

which should return ::

  is_on: False

Using the uuv_control_utils ROS node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Even though the service call can be used in run time, if you want to schedule one or more thrusters to be turned off at a specific time and, optionally,
set them to be turned on again after some time, you can use the ROS node in **uuv_control_utils**, `set_thruster_state.py <https://github.com/uuvsimulator/uuv_simulator/blob/master/uuv_control/uuv_control_utils/scripts/set_thruster_state.py>`_.

As an example, start the previous simulation again ::

  roslaunch uuv_gazebo start_pid_demo.launch

and then start the node with its launch file as ::

  roslaunch uuv_control_utils set_thruster_state.launch uuv_name:=rexrov starting_time:=10 duration:=20 is_on:=false thruster_id:=2

The argument **uuv_name** refers to the namespace of the robot model, **starting_time** is the simulation time stamp when the thruster state is going to be
altered in seconds, **duration** refers to the duration in seconds of this new thruster state (set it to -1 if the thruster should be kept in this state
indefinitely), **is_on** should be set to false to turn off the thruster and **thruster_id** is the index of the unit.

This launch file can also be included in other launch files to build a scenario with this thruster failure setup. The following example can called to
demonstrate this as ::

  roslaunch uuv_tutorial_disturbances tutorial_thruster_state.launch

You can monitor the thruster output during the simulation using **rqt_plot** or by reading the thruster unit output topic as ::

   rostopic echo /rexrov/thrusters/2/thrust.

Disturbance manager
-------------------
