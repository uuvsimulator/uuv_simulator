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

# This is a message to hold data from a DVL sensor (Doppler Velocity Log).
#
# Distances are in [m], velocities in [m/s]
#
# If the covariance is known, it should be filled.
# If it is unknown, it should be set to all zeros.
# If a measurement was invalid, its covariance should be set to -1 so it can be
# disregarded.
#
# DVLBeams are optional. If they are set they contain individual ranges and 1D
# doppler velocity estimates orthogonal to the ray.

Header header
# Measured velocity [m/s]
geometry_msgs/Vector3 velocity
# Row major, xyz axes
float64[9] velocity_covariance
# Altitude of the vehicle
float64 altitude
DVLBeam[] beams
