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
from uuv_trajectory_generator import Waypoint

class TestWaypoint(unittest.TestCase):
    def test_equal_waypoints(self):
        wp0 = Waypoint(x=1, y=2, z=3)
        wp1 = Waypoint(x=1, y=2, z=3)
        self.assertEquals(wp0, wp1, 'Waypoints equal operator failed')

    def test_unequal_waypoints(self):
        wp0 = Waypoint(x=1, y=2, z=3)
        wp1 = Waypoint(x=6, y=5, z=4)
        self.assertNotEquals(wp0, wp1, 'Waypoints unequal operator failed')

    def test_violate_constraint_flag(self):
        wp = Waypoint()
        wp.violates_constraint = True
        self.assertTrue(wp.violates_constraint, 'Constraint violation flag not correctly set')

    def test_distance_calculation(self):
        wp = Waypoint(x=0, y=2, z=0)
        self.assertEquals(wp.dist([0, 4, 0]), 2, 'Distance calculation is wrong')
        self.assertEquals(wp.dist([2, 2, 0]), 2, 'Distance calculation is wrong')
        self.assertEquals(wp.dist([0, 2, 2]), 2, 'Distance calculation is wrong')

    def test_to_message(self):
        wp0 = Waypoint(x=1, y=2, z=3)
        wp1 = Waypoint()
        wp1.from_message(wp0.to_message())
        self.assertEquals(wp0, wp1, "Waypoint to message conversion failed")


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_waypoint', TestWaypoint)
