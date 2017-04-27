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

from scipy.interpolate import splrep, splev
import numpy as np
from path_generator import PathGenerator
from ..waypoint import Waypoint
from ..waypoint_set import WaypointSet
from ..trajectory_point import TrajectoryPoint
from tf.transformations import quaternion_multiply, quaternion_about_axis


class CSInterpolator(PathGenerator):
    LABEL = 'cubic_interpolator'

    def __init__(self):
        super(CSInterpolator, self).__init__(self)

        # Set of interpolation functions for each degree of freedom
        # The heading function interpolates the given heading offset and its
        # value is added to the heading computed from the trajectory
        self._interp_fcns = dict(x=None,
                                 y=None,
                                 z=None,
                                 heading=None)

        self._splines = dict(x=None,
                             y=None,
                             z=None,
                             heading=None)

    def init_interpolator(self):
        # Generate the parametric variable for each of the interpolators
        t = 0.0
        heading = list()
        self._s = [0.0]
        # Compute the parameter vector s
        for i in range(1, self._waypoints.num_waypoints):
            wp_this = self._waypoints.get_waypoint(i)
            wp_last = self._waypoints.get_waypoint(i - 1)

            dx = wp_this.x - wp_last.x
            dy = wp_this.y - wp_last.y
            dz = wp_this.z - wp_last.z
            heading.append(wp_this.heading_offset)
            # Calculate distance of travel on a straight line
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            # Calculate the time of travel based on the set maximum
            # velocity set for this waypoint
            t += dist / wp_last.max_forward_speed
            self._s.append(t)
        # Advance parameter for the interpolator
        # Repeat the first value for the heading function
        if self._waypoints.get_waypoint(0).heading_offset is None:
            heading = [heading[0]] + heading
        else:
            heading = [self._waypoints.get_waypoint(0).heading_offset] + heading
        if self._max_time is None:
            self._max_time = t
        self._s = np.array(self._s) / self._s[-1]
        self._cur_s = 0.0

        self._splines['x'] = splrep(self._s, self._waypoints.x, k=3, per=False)
        self._interp_fcns['x'] = lambda x: splev(x, self._splines['x'])

        self._splines['y'] = splrep(self._s, self._waypoints.y, k=3, per=False)
        self._interp_fcns['y'] = lambda x: splev(x, self._splines['y'])

        self._splines['z'] = splrep(self._s, self._waypoints.z, k=3, per=False)
        self._interp_fcns['z'] = lambda x: splev(x, self._splines['z'])

        self._splines['heading'] = splrep(self._s, heading, k=3, per=False)
        self._interp_fcns['heading'] = lambda x: splev(x, self._splines['heading'])

        return True

    def get_samples(self, max_time, step=0.005):
        if self._waypoints is None:
            return None
        s = np.arange(0, 1 + step, step)
        t = s * self._max_time
        x = self._interp_fcns['x'](s)
        y = self._interp_fcns['y'](s)
        z = self._interp_fcns['z'](s)

        pnts = list()
        for i in range(t.size):
            pnt = TrajectoryPoint()
            pnt.pos = [x[i], y[i], z[i]]
            pnt.t = t[i]
            pnts.append(pnt)
        return pnts

    def generate_pos(self, s):
        pos = [self._interp_fcns['x'](s),
               self._interp_fcns['y'](s),
               self._interp_fcns['z'](s)]
        return pos

    def generate_pnt(self, s, t=0.0):
        pnt = TrajectoryPoint()
        # Trajectory time stamp
        pnt.t = t
        # Set position vector
        pnt.pos = self.generate_pos(s)
        # Set rotation quaternion
        pnt.rotq = self.generate_quat(s)
        return pnt

    def generate_quat(self, s):
        if s < 0:
            s = 0
        if s > 1:
            s = 1
        last_s = s - self._s_step
        if last_s == 0:
            last_s = 0

        dx = self._interp_fcns['x'](s) - self._interp_fcns['x'](last_s)
        dy = self._interp_fcns['y'](s) - self._interp_fcns['y'](last_s)
        dz = self._interp_fcns['z'](s) - self._interp_fcns['z'](last_s)

        rotq = self._compute_rot_quat(dx, dy, dz)

        # Calculating the step for the heading offset
        q_step = quaternion_about_axis(
            self._interp_fcns['heading'](s),
            np.array([0, 0, 1]))
        # Adding the heading offset to the rotation quaternion
        rotq = quaternion_multiply(rotq, q_step)

        return rotq
