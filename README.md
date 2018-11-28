## uuv_simulator (kinetic) - 0.6.1-0

The packages in the `uuv_simulator` repository were released into the `kinetic` distro by running `/usr/local/bin/bloom-release --rosdistro kinetic --track kinetic uuv_simulator --edit` on `Wed, 28 Nov 2018 23:57:47 -0000`

These packages were released:
- `uuv_assistants`
- `uuv_auv_control_allocator`
- `uuv_control_cascaded_pid`
- `uuv_control_msgs`
- `uuv_control_utils`
- `uuv_descriptions`
- `uuv_gazebo`
- `uuv_gazebo_plugins`
- `uuv_gazebo_ros_plugins`
- `uuv_gazebo_ros_plugins_msgs`
- `uuv_sensor_plugins_ros_msgs`
- `uuv_sensor_ros_plugins`
- `uuv_simulator`
- `uuv_teleop`
- `uuv_thruster_manager`
- `uuv_trajectory_control`
- `uuv_tutorial_disturbances`
- `uuv_tutorial_dp_controller`
- `uuv_tutorial_seabed_world`
- `uuv_tutorials`
- `uuv_world_plugins`
- `uuv_world_ros_plugins`
- `uuv_world_ros_plugins_msgs`

Version of package(s) in repository `uuv_simulator`:

- upstream repository: https://github.com/uuvsimulator/uuv_simulator.git
- release repository: unknown
- rosdistro version: `null`
- old version: `null`
- new version: `0.6.1-0`

Versions of tools used:

- bloom version: `0.6.9`
- catkin_pkg version: `0.4.8`
- rosdep version: `0.12.2`
- rosdistro version: `0.7.0`
- vcstools version: `0.1.40`


# uuv_simulator: Unmanned Underwater Vehicle (UUV) simulation with Gazebo

[![Build Status](https://travis-ci.org/uuvsimulator/uuv_simulator.svg?branch=dev%2Ftravis_integration)](https://travis-ci.org/uuvsimulator/uuv_simulator)
[![Read the Docs](https://img.shields.io/readthedocs/pip.svg)](https://uuvsimulator.github.io/)
[![Gitter](https://img.shields.io/gitter/room/nwjs/nw.js.svg)](https://gitter.im/uuvsimulator)


This package contains plugins to allow simulating UUVs with Gazebo. For installation and usage instructions, please refer to the [documentation pages](https://uuvsimulator.github.io/).
To send questions, report bugs or suggest improvements, please use the [Issues](https://github.com/uuvsimulator/uuv_simulator/issues) page.

If you are using this simulator for your publication, please cite:

```
@inproceedings{Manhaes_2016,
	doi = {10.1109/oceans.2016.7761080},
	url = {https://doi.org/10.1109%2Foceans.2016.7761080},
	year = 2016,
	month = {sep},
	publisher = {{IEEE}},
	author = {Musa Morena Marcusso Manh{\~{a}}es and Sebastian A. Scherer and Martin Voss and Luiz Ricardo Douat and Thomas Rauschenbach},
	title = {{UUV} Simulator: A Gazebo-based package for underwater intervention and multi-robot simulation},
	booktitle = {{OCEANS} 2016 {MTS}/{IEEE} Monterey}
}
```

In you are willing to contribute to this package, please check the instructions in [CONTRIBUTING](CONTRIBUTING.md)

## Purpose of the project

This software is a research prototype, originally developed for the EU ECSEL
Project 662107 [SWARMs](http://swarms.eu/).

The software is not ready for production use. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards (e.g. ISO 26262).

## License

UUV Simulator is open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.

For a list of other open source components included in UUV Simulator, see the
file [3rd-party-licenses.txt](3rd-party-licenses.txt).
