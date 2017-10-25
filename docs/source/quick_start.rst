.. _quick_start:

Quick start
===========

Start an empty underwater environment using either ::

  roslaunch uuv_descriptions empty_underwater_world.launch

or ::

  roslaunch uuv_descriptions ocean_waves.launch

Spawn the remotely operated vehicle RexROV (find the robot description files in **uuv_descriptions/models/rexrov**) as follows ::

  roslaunch uuv_descriptions upload_rexrov.launch mode:=default x:=0 y:=0 z:=-20 namespace:=rexrov

for which **mode** stands for the configuration of the vehicle to be used. It is important to create the vehicles under a unique
**namespace** to allow simulation of multiple vehicles in the same scenario.

You can start a velocity controller with a joystick teleoperation node as ::

  roslaunch uuv_control_cascaded_pid joy_velocity.launch uuv_name:=rexrov model_name:=rexrov joy_id:=0

In this case **model_name** refers to the vehicle model, which can be different from the **namespace**. It is a necessary parameter to load
the correct controller and thruster allocation matrix coefficients. The joystick ID is already set zero as default. To find the correct
joystick index, you can install and run **jstest-gtk**.

.. note::

  The mapping of the joystick teleoperation node is set as default for the XBox 360 controller. Remapping is possible by passing the correct
  indexes of the desired axes in the launch file located in the **uuv_vehicle_teleop**.

.. note::

  Sometimes Gazebo takes a while to close. Try ::

    killall -9 gzserver gzclient

  in case that happens.


Running the simulation remotely
-------------------------------

If you are having trouble running your application in multiple computers, you can try adding the hostnames and IP addresses of your machines
to the **/etc/hosts** file of the computer running **roscore**.
