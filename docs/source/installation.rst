.. _installation:

Installation
============

We assume you are using at least Ubuntu 14.04.4 LTS and ROS Indigo, even though the simulator package should also work with later versions
(minor adjustments may be required). Please refer to the instructions for `ROS Indigo installation <http://wiki.ros.org/indigo/Installation/Ubuntu>`_,
and for `ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_.

Dependencies
------------

Checkout below the needed dependencies for both ROS Indigo and Kinetic. Choose the ones you need according to the ROS version you are using.

Using UUV Simulator with ROS Indigo and Gazebo 7
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After the installation of ROS Indigo, the following packages are also needed ::

  sudo apt-get install gazebo7 libgazebo7-dev protobuf-compiler protobuf-c-compiler ros-indigo-gazebo7-msgs ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros ros-indigo-gazebo7-ros-control ros-indigo-gazebo7-ros-pkgs ros-indigo-effort-controllers ros-indigo-image-pipeline ros-indigo-image-common ros-indigo-perception ros-indigo-perception-pcl ros-indigo-robot-state-publisher ros-indigo-ros-base ros-indigo-viz python-wstool python-catkin-tools python-catkin-lint ros-indigo-hector-localization ros-indigo-joy ros-indigo-joy-teleop libopencv-dev

To use the vehicles with robotic manipulators using ROS Indigo, it might also be necessary to use a different version of the **ros-control** modules (listed below). In that case, clone the following repositories in the **src** folder ::

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


Using UUV Simulator with ROS Kinetic and Gazebo 7
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you installed ROS Kinetic, then install the following packages ::

  sudo apt-get install ros-kinetic-gazebo-msgs ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-effort-controllers ros-kinetic-image-pipeline ros-kinetic-image-common ros-kinetic-perception ros-kinetic-perception-pcl ros-kinetic-robot-state-publisher ros-kinetic-ros-base ros-kinetic-viz python-wstool python-catkin-tools python-catkin-lint ros-kinetic-hector-localization ros-kinetic-joy ros-kinetic-joy-teleop libopencv-dev protobuf-compiler protobuf-c-compiler

Creating and configuring a workspace
------------------------------------

If you don't have the ROS workspace yet, you should run the following and then clone the **uuv_simulator** package in the **~/catkin_ws/src** folder ::

  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src

Be sure to install **catkin tools** package by following the installation instructions on the `catkin tools documentation page <https://catkin-tools.readthedocs.io/en/latest/installing.html>`_. After the installation, initialize the catkin workspace ::

  cd ~/catkin_ws
  catkin init

You can then clone the UUV simulator into your **src** folder ::

  cd ~/catkin_ws/src
  git clone https://github.com/uuvsimulator/uuv_simulator.git

Configure the environment variables by adding the following lines in **~/.bashrc** (replace **kinetic** with **indigo** depending on the ROS version you  are using).

.. note::

    If you install a version of Gazebo newer than 7.0, you might need to adjust **gazebo-7.0** below.
    You can find out which version you are using by typing ::

      gazebo --version

    in your terminal.

::

  source /usr/share/gazebo-7/setup.sh
  source /opt/ros/kinetic/setup.bash
  source $HOME/catkin_ws/devel/setup.sh

  export GAZEBO_PREFIX=$HOME/catkin_ws/install
  export GAZEBO_RESOURCE_PATH=${GAZEBO_PREFIX}/share/gazebo-7.0:${GAZEBO_RESOURCE_PATH}
  export GAZEBO_MODEL_PATH=${GAZEBO_PREFIX}/share/gazebo-7.0/models:${GAZEBO_MODEL_PATH}
  export GAZEBO_PLUGIN_PATH=${GAZEBO_PREFIX}/lib:${GAZEBO_PREFIX}/lib/x86_64-linux-gnu:${GAZEBO_PLUGIN_PATH}

After saving these changes, remember to source the **.bashrc** by either typing ::

  source ~/.bashrc

in your terminal or reopening the terminal. Finally, clone the UUV simulator package in the folder **~/catkin_ws/src** and build your workspace using ::

  cd ~/catkin_ws
  catkin_make install

or ::

  cd ~/catkin_ws
  catkin build

in case you are using **catkin_tools**.

.. note:: 

  If after compiling your catkin workspace using **catkin build** ROS seems to not update the paths to the packages even after you run ::

    cd ~/catkin_ws
    source devel/setup.sh

  you can try disabling the option to source the **install** folder of your catkin workspace by running ::

    cd ~/catkin_ws
    catkin config --no-install
    catkin clean --all

  Then rebuild your workspace ::

    catkin build
    source devel/setup.sh