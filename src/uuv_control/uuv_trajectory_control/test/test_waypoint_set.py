#!/usr/bin/env python
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

import roslib

PKG = 'uuv_trajectory_control'
roslib.load_manifest(PKG)

import sys
import unittest
from uuv_waypoints import Waypoint, WaypointSet


class TestWaypointSet(unittest.TestCase):
    def test_init(self):
        wp_set = WaypointSet()
        self.assertEquals(wp_set.num_waypoints, 0, 'Waypoint list is not empty')

    def test_invalid_params_helix(self):
        wp_set = WaypointSet()
        self.assertFalse(wp_set.generate_helix(
            radius=-1.0,
            center=None,
            num_points=-1,
            max_forward_speed=0.0,
            delta_z=1,
            num_turns=-1,
            theta_offset=0.0,
            heading_offset=0.0), 'Invalid parameters have been wrongly instantiated')

    def test_invalid_params_circle(self):
        wp_set = WaypointSet()
        self.assertFalse(wp_set.generate_circle(
            radius=-1,
            center=None,
            num_points=-1,
            max_forward_speed=0,
            theta_offset=0.0,
            heading_offset=0.0), 'Invalid parameters have been wrongly instantiated')

    def test_add_repeated_waypoint(self):
        wp = Waypoint(x=1, y=2, z=3, max_forward_speed=1)
        wp_set = WaypointSet()
        self.assertTrue(wp_set.add_waypoint(wp),
            'Error occured while adding waypoint to empty set')
        self.assertFalse(wp_set.add_waypoint(wp),
            'Repeated waypoint wrongfully added')


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_waypoint_set', TestWaypointSet)
