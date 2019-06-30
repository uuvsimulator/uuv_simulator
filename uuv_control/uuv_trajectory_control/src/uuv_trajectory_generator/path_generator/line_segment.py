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


class LineSegment(object):
    """Line segment class.
    
    > *Input arguments*
    
    * `p_init` (*type:* `list` or `numpy.array`): Line's starting point
    * `p_target` (*type:* `list` or `numpy.array`): Line's ending point
    """
    def __init__(self, p_init, p_target):
        if type(p_init) == list:
            assert len(p_init) == 3, 'Initial segment point must have 3 elements'
            self._p_init = np.array(p_init)
        elif type(p_init) == np.ndarray:
            assert p_init.size == 3, 'Initial segment point must have 3 elements'
            self._p_init = p_init
        else:
            raise TypeError('Initial point is neither a list or an array')

        if type(p_target) == list:
            assert len(p_target) == 3, 'Final segment point must have 3 elements'
            self._p_target = np.array(p_target)
        elif type(p_target) == np.ndarray:
            assert p_target.size == 3, 'Final segment point must have 3 elements'
            self._p_target = p_target
        else:
            raise TypeError('Final point is neither a list or an array')

        assert not np.array_equal(self._p_init, self._p_target), 'Initial and final points are equal'

    def interpolate(self, u):
        """Interpolate the Bezier curve using the input parametric variable `u`.
        
        > *Input arguments*
        
        * `u` (*type:* `float`): Curve parametric input in the interval `[0, 1]`
        
        > *Returns*
        
        `numpy.array`: 3D point from the Bezier curve
        """
        u = max(u, 0)
        u = min(u, 1)
        return (1 - u) * self._p_init + u * self._p_target

    def get_derivative(self, *args):        
        """Compute the derivative of the line segment.

        > *Returns*
        
        `numpy.array`: 3D derivative value from the Bezier curve
        """
        return self._p_target - self._p_init

    def get_length(self):
        """Get length of the Bezier curve segment.

        > *Returns*
        
        `float`: Length of the curve
        """
        return np.linalg.norm(self._p_target - self._p_init)

    def get_tangent(self):
        """Compute tangent vector.
        
        > *Returns*
        
        `numpy.array`: Tangent vector
        """
        return (self._p_target - self._p_init) / self.get_length()


