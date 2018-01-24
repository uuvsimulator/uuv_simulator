#!/bin/bash
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

sudo apt -qq install --no-install-recommends --allow-unauthenticated -y \
  gazebo7 libgazebo7-dev protobuf-compiler protobuf-c-compiler \
  ros-indigo-gazebo7-msgs ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros \
  ros-indigo-gazebo7-ros-control ros-indigo-gazebo7-ros-pkgs \
  ros-indigo-effort-controllers ros-indigo-image-pipeline ros-indigo-image-common \
  ros-indigo-perception ros-indigo-perception-pcl ros-indigo-robot-state-publisher \
  ros-indigo-ros-base ros-indigo-viz python-wstool python-catkin-tools python-catkin-lint \
  ros-indigo-hector-localization ros-indigo-joy ros-indigo-joy-teleop libopencv-dev

source /usr/share/gazebo-7/setup.sh

sh .uuv_ci_config/uuv_dependencies.sh
