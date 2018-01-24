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

Header header
# Sensor link position
geometry_msgs/Vector3 position
# Latitude in degrees (WGS84 coordinates)
float64 latitude
# Longiture in degrees (WGS84 coordinates)
float64 longitude
# Depth in meters (WGS84 coordinates)
float64 depth
# Particle concentration
float64 concentration
# Status, false if no plume particles are received
bool is_measuring
