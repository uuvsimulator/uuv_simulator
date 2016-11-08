# uuv_simulator: Unmanned Underwater Vehicle (UUV) simulation with gazebo

## Overview

This package contains add-ons to allow simulating UUVs with gazebo. The overall
structure is:

 - ```uuv_gazebo```: Contains only launch files and visualization options of
   demos to showcase the simulator.

 - ```uuv_gazebo_plugins```: Contains Gazebo plugins to simulate underwater
   physics, e.g. added mass, damping, UVV thrusters and fins. These plugins are
   middleware-independent, i.e. they can be used without ROS.

 - ```uuv_gazebo_plugins_ros```: Derived plugins that allow communication with
   ROS.

 - ```uuv_sensor_plugins```: Gazebo plugins to simulate underwater sensors.
   These plugins are middleware-agnostic, i.e. they can be used without ROS.

 - ```uuv_sensor_plugins_ros```: Derived plugins that allow communication with
   ROS.

 - ```uuv_descriptions```: Gazebo vehicles models, world models, and worlds.

 - ```uuv_control```: Basic example controllers for both ROVs and AUVs.

 - ```uuv_teleop```: Interfaces to remotely control ROVs and AUVs.

 - ```uuv_manipulators```: Everything related to manipulators.


You can cite the following paper if you use UUV Simulator for a scientific
publication:

```
@inproceedings{Marcusso_2016,
  author={Marcusso Manh{\~{a}}es, Musa Morena and Scherer, Sebastian A. and
          Voss, Martin and Douat, Luiz Ricardo and Rauschenbach, Thomas},
  booktitle={OCEANS 2016 - MTS/IEEE Monterey},
  title={UUV Simulator: A Gazebo-based Package for Underwater Intervention and
         Multi-Robot Simulation},
  year={2016}
}
```

## Requirements

We assume you are using Ubuntu 14.04.4 LTS and ROS indigo, even though the
simulator package should also work with later versions (minor adjustments may be
required).

We recommend you install the following Gazebo packages from the OSRF repository
as described in [this Gazebo tutorial](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

 - gazebo7
 - libgazebo7-dev
 - protobuf-compiler
 - protobuf-c-compiler
 - ros-indigo-gazebo7-msgs
 - ros-indigo-gazebo7-plugins
 - ros-indigo-gazebo7-ros
 - ros-indigo-gazebo7-ros-control
 - ros-indigo-gazebo7-ros-pkgs

You should also install the following ROS packages:

 - ros-indigo-effort-controllers
 - ros-indigo-image-pipeline
 - ros-indigo-image-common
 - ros-indigo-moveit-full
 - ros-indigo-perception
 - ros-indigo-perception-pcl
 - ros-indigo-robot-state-publisher
 - ros-indigo-ros-base
 - ros-indigo-viz
 - python-wstool
 - python-catkin-tools
 - python-catkin-lint
 - ros-indigo-hector-localization
 - ros-indigo-joy
 - ros-indigo-joy-teleop

## Compilation & Installation

Check out uuv_simulator within the src directory of a catkin workspace.
You should also check out the following additional packages:

```bash
git clone https://github.com/ros-controls/control_msgs.git
cd control_msgs
git checkout c0b322b
cd ..

git clone https://github.com/ros-controls/control_toolbox.git
cd control_toolbox
git checkout 5ccdc6d
cd ..

git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
cd gazebo_ros_pkgs
git checkout 231b76d
cd ..

git clone https://github.com/ros-controls/realtime_tools.git
cd realtime_tools
git checkout bf55298
cd ..

git clone https://github.com/ros-controls/ros_controllers.git
cd ros_controllers
git checkout b4dc152
cd ..
```

Compile and install the package using the following command:

```catkin_make install```

Then set the following environment variables within your .bashrc:

```bash
source /usr/share/gazebo-7/setup.sh
export GAZEBO_PREFIX=$HOME/code/ws/install
export GAZEBO_RESOURCE_PATH=${GAZEBO_PREFIX}/share/gazebo-7.3:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=${GAZEBO_PREFIX}/share/gazebo-7.3/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=${GAZEBO_PREFIX}/lib:${GAZEBO_PREFIX}/lib/x86_64-linux-gnu:${GAZEBO_PLUGIN_PATH}
```

Note: If you install a  version of gazebo newer than 7.3, you might need to
adjust gazebo-7.3 above.

## Example usage

Start the simulation with an empty underwater environment:

```roslaunch uuv_descriptions heightmap.launch```

Spawn a remotely operated vehicle (ROV):

```roslaunch uuv_gazebo rexrov_manipulation.launch```

You can then control both the ROV and the manipulator using an XBOX360 gamepad.

Or you can start a world as before and upload a vehicle without controllers as
follows:

```roslaunch uuv_descriptions upload_rexrov.launch mode:=default x:=0 y:=0 z:=0 namespace:=rexrov```

It is important to give each vehicle an unique namespace name, as this will be
its identifier.

To start a velocity control node with a joystick input, start:

```roslaunch uuv_control_cascaded_pid joy_velocity.launch uuv_name:=rexrov model_name:=rexrov joy_id:=0```

In this case ```model_name``` refers to the vehicle model, which can be
different from the ```namespace```. It is a necessary parameter to load the
correct controller and thruster allocation matrix coefficients. The joystick ID
is already set zero as default.

## License

UUV Simulator is open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.

For a list of other open source components included in UUV Simulator, see the
file [3rd-party-licenses.txt](3rd-party-licenses.txt).
