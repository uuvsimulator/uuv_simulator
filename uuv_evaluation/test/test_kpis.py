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

PKG = 'uuv_evaluation'
NAME = 'test_kpis'

import rospy
import rostest
import unittest
import numpy as np
from bag_evaluation.metrics import KPI
import roslib; roslib.load_manifest(PKG)


class TestKPIS(unittest.TestCase):
    def test_max_error_without_bag(self):
        error = [1.0 for _ in range(10)]
        max_error = 20.0
        error[2] = max_error

        # Not using a ROS bag to calculate the KPIs
        kpi = KPI.get_kpi('max_error', 'test', False)
        self.assertIsNotNone(kpi, 'The KPI instance was not created properly')
        self.assertEqual(kpi.compute(error), max_error, 'Max. error was not calculated properly')

    def test_mean_error_without_bag(self):
        error = np.random.rand(10) + 2
        # Not using a ROS bag to calculate the KPIs
        kpi = KPI.get_kpi('mean_error', 'test', False)
        self.assertIsNotNone(kpi, 'The KPI instance was not created properly')
        self.assertIsNotNone(kpi.compute(error), 'Mean error was not calculated properly')

    def test_rms_error_without_bag(self):
        error = np.random.rand(10) + 2
        # Not using a ROS bag to calculate the KPIs
        kpi = KPI.get_kpi('rmse', 'test', False)
        self.assertIsNotNone(kpi, 'The KPI instance was not created properly')
        self.assertIsNotNone(kpi.compute(error), 'RMS error was not calculated properly')
        
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestKPIS)