#!/usr/bin/env python
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

import roslib

PKG = 'uuv_trajectory_control'
roslib.load_manifest(PKG)

import sys
import unittest
import numpy as np
from uuv_trajectory_generator import TrajectoryPoint
from uuv_control_msgs.msg import TrajectoryPoint as TrajectoryPointMsg


class TestTrajectoryPoint(unittest.TestCase):
    def test_init_pos_vector(self):
        p = TrajectoryPoint()
        self.assertEquals(p.pos.size, 3, 'Position vector len() is incorrect')
        self.assertTrue(np.array_equal(p.pos, [0, 0, 0]), 'Position initialization failed')

    def test_set_pos_vector(self):
        p = TrajectoryPoint()
        p.pos = [1, 2, 3]
        self.assertEquals(p.pos[0], 1, 'X position was initialized wrong')
        self.assertEquals(p.pos[1], 2, 'Y position was initialized wrong')
        self.assertEquals(p.pos[2], 3, 'Z position was initialized wrong')

    def test_init_quat_vector(self):
        p = TrajectoryPoint()
        self.assertEquals(p.rotq.size, 4, 'Quaternion vector len() is incorrect')
        self.assertTrue(np.array_equal(p.rotq, [0, 0, 0, 1]), 'Quaternion initialization failed')

    def test_to_message(self):
        p0 = TrajectoryPoint()
        p0.t = 1
        p0.pos = [1, 2, 3]
        p0.rotq = [0, 0, 1, 1]
        p0.vel = [1, 2, 3, 4, 5, 6]
        p0.acc = [1, 2, 3, 4, 5, 6]

        p1 = TrajectoryPoint()
        p1.from_message(p0.to_message())

        self.assertEquals(p0, p1, 'Point to message conversion failed')

    def test_to_dict(self):
        p0 = TrajectoryPoint()
        p0.t = 1
        p0.pos = [1, 2, 3]
        p0.rotq = [0, 0, 1, 1]
        p0.vel = [1, 2, 3, 4, 5, 6]
        p0.acc = [1, 2, 3, 4, 5, 6]

        p1 = TrajectoryPoint()
        p1.from_dict(p0.to_dict())

        self.assertEquals(p0, p1, 'Point to dict conversion failed')

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_trajectory_point', TestTrajectoryPoint)
