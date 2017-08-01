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

from copy import deepcopy
from scipy.interpolate import splrep, splev
import numpy as np
from ..waypoint import Waypoint
from ..waypoint_set import WaypointSet
from ..trajectory_point import TrajectoryPoint
from tf.transformations import quaternion_multiply, quaternion_about_axis
from line_segment import LineSegment
from bezier_curve import BezierCurve
from path_generator import PathGenerator


class LIPBInterpolator(PathGenerator):
    """
    Linear interpolator with polynomial blends.

    [1] Biagiotti, Luigi, and Claudio Melchiorri. Trajectory planning for
        automatic machines and robots. Springer Science & Business Media, 2008.
    """
    LABEL = 'lipb_interpolator'

    def __init__(self):
        super(LIPBInterpolator, self).__init__(self)

        self._radius = 5
        # Set of interpolation functions for each degree of freedom
        # The heading function interpolates the given heading offset and its
        # value is added to the heading computed from the trajectory
        self._interp_fcns = dict(pos=None,
                                 heading=None)
        self._heading_spline = None

    def init_interpolator(self):
        if self._waypoints is None:
            return False

        self._interp_fcns['pos'] = list()
        self._segment_to_wp_map = [0]
        
        if self._waypoints.num_waypoints == 2:
            self._interp_fcns['pos'].append(
                LineSegment(self._waypoints.get_waypoint(0).pos,
                            self._waypoints.get_waypoint(1).pos))
            self._segment_to_wp_map.append(1)
            # Set a simple spline to interpolate heading offset, if existent
            heading = [self._waypoints.get_waypoint(k).heading_offset for k in range(self._waypoints.num_waypoints)]

        elif self._waypoints.num_waypoints > 2:
            q_seg = self._waypoints.get_waypoint(0).pos
            q_start_line = q_seg
            heading = [0]
            for i in range(1, self._waypoints.num_waypoints):
                first_line = LineSegment(q_start_line, self._waypoints.get_waypoint(i).pos)
                radius = min(self._radius, first_line.get_length() / 2)
                if i + 1 < self._waypoints.num_waypoints:
                    second_line = LineSegment(self._waypoints.get_waypoint(i).pos,
                                              self._waypoints.get_waypoint(i + 1).pos)
                    radius = min(radius, second_line.get_length() / 2)
                if i < self._waypoints.num_waypoints - 1:
                    q_seg = np.vstack(
                        (q_seg, first_line.interpolate((first_line.get_length() - radius) / first_line.get_length())))
                    self._interp_fcns['pos'].append(LineSegment(q_start_line, q_seg[-1, :]))
                    heading.append(0)
                    self._segment_to_wp_map.append(i)
                if i == self._waypoints.num_waypoints - 1:
                    q_seg = np.vstack((q_seg, self._waypoints.get_waypoint(i).pos))
                    self._interp_fcns['pos'].append(LineSegment(q_seg[-2, :], q_seg[-1, :]))
                    heading.append(0)
                    self._segment_to_wp_map.append(i)
                elif i + 1 < self._waypoints.num_waypoints:
                    q_seg = np.vstack((q_seg, second_line.interpolate(radius / second_line.get_length())))
                    self._interp_fcns['pos'].append(
                        BezierCurve([q_seg[-2, :], self._waypoints.get_waypoint(i).pos, q_seg[-1, :]], 5))
                    heading.append(0)
                    self._segment_to_wp_map.append(i)
                    q_start_line = deepcopy(q_seg[-1, :])
        else:
            return False
        
        # Reparametrizing the curves
        lengths = [seg.get_length() for seg in self._interp_fcns['pos']]
        lengths = [0] + lengths
        self._s = np.cumsum(lengths) / np.sum(lengths)

        mean_vel = np.mean(
            [self._waypoints.get_waypoint(k).max_forward_speed for k in range(self._waypoints.num_waypoints)])
        if self._duration is None:
            self._duration = np.sum(lengths) / mean_vel
        if self._start_time is None:
            self._start_time = 0.0

        if self._waypoints.num_waypoints == 2:
            head_offset_line = deepcopy(self._waypoints.get_waypoint(1).heading_offset)
            self._interp_fcns['heading'] = lambda x: head_offset_line
        else:
            # Set a simple spline to interpolate heading offset, if existent
            self._heading_spline = splrep(self._s, heading, k=3, per=False)
            self._interp_fcns['heading'] = lambda x: splev(x, self._heading_spline)

        return True

    def get_samples(self, max_time, step=0.005):
        if self._waypoints is None:
            return None
        s = np.arange(0, 1 + step, step)
        t = s * self._duration + self._start_time

        pnts = list()
        for i in range(t.size):
            pnt = TrajectoryPoint()
            pnt.pos = self.generate_pos(s[i]).tolist()
            pnt.t = t[i]
            pnts.append(pnt)
        return pnts

    def generate_pos(self, s):
        idx = self.get_segment_idx(s)
        if idx == 0:
            u_k = 0
            pos = self._interp_fcns['pos'][idx].interpolate(u_k)
        else:
            u_k = (s - self._s[idx - 1]) / (self._s[idx] - self._s[idx - 1])
            pos = self._interp_fcns['pos'][idx - 1].interpolate(u_k)
        return pos

    def generate_pnt(self, s, t=0.0):
        pnt = TrajectoryPoint()
        # Trajectory time stamp
        pnt.t = t
        # Set position vector
        pnt.pos = self.generate_pos(s).tolist()
        # Set rotation quaternion
        pnt.rotq = self.generate_quat(s)
        return pnt

    def generate_quat(self, s):
        s = max(0, s)
        s = min(s, 1)

        last_s = s - self._s_step
        if last_s == 0:
            last_s = 0

        this_pos = self.generate_pos(s)
        last_pos = self.generate_pos(last_s)
        dx = this_pos[0] - last_pos[0]
        dy = this_pos[1] - last_pos[1]
        dz = this_pos[2] - last_pos[2]

        rotq = self._compute_rot_quat(dx, dy, dz)

        # Calculating the step for the heading offset
        q_step = quaternion_about_axis(
            self._interp_fcns['heading'](s),
            np.array([0, 0, 1]))
        # Adding the heading offset to the rotation quaternion
        rotq = quaternion_multiply(rotq, q_step)

        return rotq
