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

float64 range    # measured range [m] or < 0 if invalid
float64 range_covariance

float64 velocity # measured velocity [m/s] of corr. beam
float64 velocity_covariance

float64 yaw      # yaw angle [rad] of corresponding beam
float64 pitch    # pitch angle [rad] of corresponding beam
