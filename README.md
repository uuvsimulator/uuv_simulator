# `uuv_simulator`: Unmanned Underwater Vehicle (UUV) simulation with Gazebo

[![Build Status](https://travis-ci.org/uuvsimulator/uuv_simulator.svg?branch=dev%2Ftravis_integration)](https://travis-ci.org/uuvsimulator/uuv_simulator)
[![GitHub issues](https://img.shields.io/github/issues/uuvsimulator/uuv_simulator.svg)](https://github.com/uuvsimulator/uuv_simulator/issues)
[![License](https://img.shields.io/badge/license-Apache%202-blue.svg)](https://github.com/uuvsimulator/uuv_simulator/blob/master/LICENSE)

> Link to the `uuv_simulator` repository [here](https://github.com/uuvsimulator/uuv_simulator)

> Link to the [documentation page](https://uuvsimulator.github.io/packages/uuv_simulator/intro/) 

> Chat on [Discord](https://discord.gg/zNauF2F)

The **Unmanned Underwater Vehicle Simulator** is a set of packages that include plugins and ROS applications that allow simulation of underwater vehicles in [Gazebo](http://gazebosim.org/). 

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

In you are willing to contribute to this package, please check the instructions in [CONTRIBUTING](https://github.com/uuvsimulator/uuv_simulator/blob/master/CONTRIBUTING.md).

# Features

> **Gazebo/ROS plugins**
  
- Implementation of Fossen's equations of motion for underwater vehicles
- Thruster modules with implementations for thruster's angular velocity to output thrust force based on [`Yoerger el al., 1990`](http://dx.doi.org/10.1109/48.107145) and [`Bessa et al., 2006`](http://www.abcm.org.br/symposium-series/SSM_Vol2/Section_IX_Submarine_Robotics/SSM2_IX_01.pdf)
- Lift and drag plugin for simulation of fins
- Simulation of 3D current velocity models (constant or based on first-order Gauss-Markov processes)
- Sensor plugins

> **Controllers**

- For AUVs
    - [`casadi`](https://web.casadi.org/)-based effort allocation algorithm 
    - Geometric tracking PD controller
- For ROVs
    - Thruster manager with computation of the thruster allocation matrix based on the thruster frames available in `/tf`
    - Model-based feedback linearization controller ([`Fossen, 2011`](https://www.wiley.com/en-us/Handbook+of+Marine+Craft+Hydrodynamics+and+Motion+Control-p-9781119991496))
    - Nonlinear PID controller ([`Fossen, 2011`](https://www.wiley.com/en-us/Handbook+of+Marine+Craft+Hydrodynamics+and+Motion+Control-p-9781119991496))
    - Non-model-based sliding mode controller ([`García-Valdovinos el al., 2014`](https://journals.sagepub.com/doi/full/10.5772/56810) and [`Salgado-Jiménez et al., 2011`](http://cdn.intechopen.com/pdfs/15221.pdf))
    - PD controller with restoration forces compensation 
    - 6-DOF PID controller
    - Singularity-free tracking controller ([`Fjellstad and Fossen, 1994`](https://ieeexplore.ieee.org/abstract/document/411068))
- Teleoperation nodes for AUVs and ROVs

> **Gazebo world models**

- Ocean wave shaders for wave animation
- Scenarios from the SWARMs project demonstration locations (e.g. Mangalia, Romania and Trondheim, Norway)
- Subsea BOP panel for manipulation tasks

> **Vehicle models**

- Work-class ROV `rexrov` based on the model presetend in [`Berg, 2012`](https://brage.bibsys.no/xmlui/handle/11250/238170?locale-attribute=no)
- [`eca_a9`](https://github.com/uuvsimulator/eca_a9)
- [`lauv_gazebo`](https://github.com/uuvsimulator/lauv_gazebo)
- [`desistek_saga`](https://github.com/uuvsimulator/desistek_saga)
- [`rexrov2`](https://github.com/uuvsimulator/rexrov2)
  
# Installation

This packages has been released for the following ROS distributions

- `kinetic` (See [installation instructions for ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu))
- `lunar` (See [installation instructions for ROS Lunar](https://wiki.ros.org/lunar/Installation/Ubuntu))
- `melodic` (See [installation instructions for ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu))

Once the `ros-<distro>-desktop-full` package for the desired distribution is installed, the uuv_simulator can be installed as

```bash tab="kinetic"
sudo apt install ros-kinetic-uuv-simulator
```

```bash tab="lunar"
sudo apt install ros-lunar-uuv-simulator
```

```bash tab="melodic"
sudo apt install ros-melodic-uuv-simulator
```

For instructions on how to install the package from source, check this [instructions page](https://uuvsimulator.github.io/installation/)

# Purpose of the project

This software is a research prototype, originally developed for the EU ECSEL
Project 662107 [SWARMs](http://swarms.eu/).

The software is not ready for production use. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards (e.g. ISO 26262).

# License

UUV Simulator is open-sourced under the Apache-2.0 license. See the
[LICENSE](https://github.com/uuvsimulator/uuv_simulator/blob/master/LICENSE) file for details.

For a list of other open source components included in UUV Simulator, see the
file [3rd-party-licenses.txt](https://github.com/uuvsimulator/uuv_simulator/blob/master/3rd-party-licenses.txt).

# Releases

[![ROS Kinetic](https://img.shields.io/badge/ROS%20Distro-kinetic-brightgreen.svg)](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=uuv_simulator)
[![ROS Lunar](https://img.shields.io/badge/ROS%20Distro-lunar-brightgreen.svg)](http://repositories.ros.org/status_page/ros_lunar_default.html?q=uuv_simulator)
[![ROS Melodic](https://img.shields.io/badge/ROS%20Distro-melodic-brightgreen.svg)](http://repositories.ros.org/status_page/ros_melodic_default.html?q=uuv_simulator)