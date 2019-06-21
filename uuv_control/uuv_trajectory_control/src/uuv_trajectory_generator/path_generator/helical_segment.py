# Copyright (c) 2016-2019 The UUV Simulator Authors.
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
import numpy as np

class HelicalSegment(object):
    """Generator of helical segments.
    
    > *Input arguments*
    
    * `center` (*type:* `list`): Center of the helix in meters
    * `radius` (*type:* `float`): Radius of the helix in meters
    * `n_turns` (*type:* `int`): Number of turns
    * `delta_z` (*type:* `float`): Length of the step in the Z direction between each turn of the helix in meters
    * `angle_offset` (*type:* `float`): Angle offset to start the helix
    * `is_clockwise` (*type:* `bool`, *default:* `True`): If `True`, the helix is generated clockwise.

    > *Example*

    ```python
    radius = 3
    center = [2, 2, 2]
    n_turns = 2
    delta_z = 1
    angle_offset = 0.0
    is_clockwise = True

    helix = HelicalSegment(center, radius, n_turns, delta_z, angle_offset, is_clockwise)

    u = numpy.linspace(0, 1, 100)
    pnts = numpy.array([helix.interpolate(i) for i in u])
    ```
    """
    def __init__(self, center, radius, n_turns, delta_z, angle_offset, is_clockwise=True):
        self._center = np.array(center)
        assert self._center.size == 3, 'Size of center point vector must be 3'

        assert radius > 0, 'Helix radius must be greater than zero'
        assert n_turns > 0, 'Number of turns must be greater than zero'
        assert isinstance(is_clockwise, bool), 'is_clockwise flag must be a boolean'
        
        self._radius = radius
        self._n_turns = n_turns
        self._angle_offset = angle_offset
        self._is_clockwise = is_clockwise
        self._delta_z = delta_z
        self._step_z = float(self._delta_z) / self._n_turns

    def get_length(self):
        """Return the length of the helix in meters"""
        return self._n_turns * np.sqrt(self._step_z**2 + (2 * np.pi * self._radius)**2)

    def get_pitch(self):
        """Return the pitch angle of the helical path in radians"""
        return np.sin(self._step_z / np.sqrt(self._step_z**2 + (2 * np.pi * self._radius)**2))

    def interpolate(self, u):
        """Compute the 3D point on the helical path 
        
        > *Input arguments*
        
        * `param` (*type:* `data_type`, *default:* `data`): Parameter description
        
        > *Returns*
        
        Description of return values
        """
        u = max(u, 0)
        u = min(u, 1)
        delta = 1 if self._is_clockwise else -1        
        x = self._radius * np.cos(self._n_turns * 2 * np.pi * u * delta + self._angle_offset)
        y = self._radius * np.sin(self._n_turns * 2 * np.pi * u * delta + self._angle_offset)
        z = self._n_turns * u * self._step_z

        return self._center + np.array([x, y, z])

