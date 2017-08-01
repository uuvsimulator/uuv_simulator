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

import rospy
import scipy
import numpy as np
from copy import deepcopy
from ..waypoint import Waypoint
from ..waypoint_set import WaypointSet
from ..trajectory_point import TrajectoryPoint
from tf.transformations import quaternion_multiply, quaternion_inverse, quaternion_from_euler, quaternion_conjugate, quaternion_about_axis


class PathGenerator(object):
    """
    Abstract class to be inherited by custom path generator to interpolate
    waypoints
    """
    LABEL = ''

    def __init__(self, full_dof=False):
        # Waypoint set
        self._waypoints = None

        # True if the path is generated for all degrees of freedom, otherwise
        # the path will be generated for (x, y, z, yaw) only
        self._is_full_dof = full_dof

        # The parametric variable to use as input for the interpolator
        self._s = list()
        self._segment_to_wp_map = list()
        self._cur_s = 0
        self._s_step = 0.0001

        self._start_time = None
        self._duration = None

    @staticmethod
    def get_generator(name, *args, **kwargs):
        for gen in PathGenerator.__subclasses__():
            if name == gen.LABEL:
                return gen(*args, **kwargs)
        rospy.ROSException('Invalid path generator method')

    @staticmethod
    def get_all_generators():
        generators = list()
        for gen in PathGenerator.__subclasses__():
            generators.append(gen())
        return generators

    @property
    def waypoints(self):
        return self._waypoints

    @property
    def max_time(self):
        return self._duration + self._start_time

    @property
    def duration(self):
        return self._duration

    @duration.setter
    def duration(self, t):
        assert t > 0, 'Duration must be a positive value'
        self._duration = t

    @property
    def start_time(self):
        return self._start_time

    @start_time.setter
    def start_time(self, time):
        assert time >= 0, 'Invalid negative time'
        self._start_time = time

    @property
    def closest_waypoint(self):
        """Return the closest waypoint to the current position on the path."""
        return self._waypoints.get_waypoint(self.closest_waypoint_idx)

    @property
    def closest_waypoint_idx(self):
        """
        Return the index of the closest waypoint to the current position on the
        path.
        """

        if self._cur_s == 0:
            return 0
        if self._cur_s == 1:
            return len(self._s) - 1
        v = np.array(self._s - self._cur_s)
        idx = np.argmin(v)
        return idx

    @property
    def s_step(self):
        return self._s_step

    @s_step.setter
    def s_step(self, step):
        assert 0 < step < 1
        self._s_step = step

    def reset(self):
        self._s = list()
        self._segment_to_wp_map = list()
        self._cur_s = 0
        self._s_step = 0.0001

        self._start_time = None
        self._duration = None

    def get_segment_idx(self, s):
        if len(self._s) == 0:
            return 0
        # Ensure the parameter s is 0 <= s <= 1
        s = max(0, s)
        s = min(s, 1)

        if s == 1:
            idx = self._s.size - 1
        else:
            idx = (self._s - s >= 0).nonzero()[0][0]
        return idx

    def get_remaining_waypoints_idx(self, s):
        idx = self.get_segment_idx(s)
        try:
            wps = self._segment_to_wp_map[idx::]
            return np.unique(wps)
        except:
            print 'Invalid index'
            return None

    def is_full_dof(self):
        return self._is_full_dof

    def set_full_dof(self, flag):
        self._is_full_dof = flag

    def get_label(self):
        return self.LABEL

    def init_interpolator(self):
        raise NotImplementedError()

    def get_samples(self, max_time, step=0.005):
        raise NotImplementedError()

    def add_waypoint(self, waypoint, add_to_beginning=False):
        """Add waypoint to the existing waypoint set. If no waypoint set has
        been initialized, create new waypoint set structure and add the given
        waypoint."""
        if self._waypoints is None:
            self._waypoints = WaypointSet()
        self._waypoints.add_waypoint(waypoint, add_to_beginning)
        return self.init_interpolator()

    def init_waypoints(self, waypoints=None):
        if waypoints is not None:
            self._waypoints = deepcopy(waypoints)

        if self._waypoints is None:
            print 'Waypoint list has not been initialized'
            return False

        return self.init_interpolator()

    def interpolate(self, tag, s):
        return self._interp_fcns[tag](s)

    def generate_pnt(self, s):
        raise NotImplementedError()

    def generate_pos(self, s):
        raise NotImplementedError()

    def generate_quat(self, s):
        raise NotImplementedError()

    def _compute_rot_quat(self, dx, dy, dz):
        rotq = quaternion_about_axis(
            np.arctan2(dy, dx),
            [0, 0, 1])

        if self._is_full_dof:
            rote = quaternion_about_axis(
                -1 * np.arctan2(dz, np.sqrt(dx**2 + dy**2)),
                [0, 1, 0])
            rotq = quaternion_multiply(rotq, rote)
        return rotq
