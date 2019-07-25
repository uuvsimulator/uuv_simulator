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
import rospy
import roslib
import numpy as np
import unittest
import time
from geometry_msgs.msg import WrenchStamped
from uuv_thruster_manager.srv import GetThrusterManagerConfig, ThrusterManagerInfo
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
# Other imports

PKG = 'uuv_thruster_manager'
roslib.load_manifest(PKG)

NS = 'test_vehicle'

AXIS_X_TAM = np.array([
    [1, 0, 0, 0, 0, 0],
    [0.87758256, 0, -0.47942554, 0.47942554, 0.47942554, 0.87758256],
    [0.87758256, 0.47942554, 0, -0.47942554, 0.87758256, -0.87758256]
]).T

AXIS_Y_TAM = np.array([
    [0, 0.87758256, 0.47942554, 0, 0.47942554, -0.87758256],
    [0, 1, 0, 0, 0, 1],
    [-0.47942554, 0.87758256, 0, -0.87758256, -0.47942554, 0.47942554]
]).T

AXIS_Z_TAM = np.array([
    [0, -0.47942554, 0.87758256, 0, 0.87758256, 0.47942554],
    [0.47942554, 0, 0.87758256, -0.87758256, -0.87758256, 0.47942554],
    [0., 0., 1., 1., 0., 0.]
]).T

class TestThrusterAllocator(unittest.TestCase):
    def test_services_exist(self):
        srvs = [
            'thruster_manager/get_thrusters_info',
            'thruster_manager/get_thruster_curve',
            'thruster_manager/set_config',
            'thruster_manager/get_config'
        ]

        for srv in srvs:
            rospy.wait_for_service('/{}/{}'.format(NS, srv), timeout=1000)

    def test_config(self):
        axis = rospy.get_param('axis')
        ref_config = rospy.get_param('/{}/thruster_manager'.format(NS))
        
        rospy.wait_for_service('/{}/thruster_manager/get_config'.format(NS), timeout=1000)
        srv = rospy.ServiceProxy('/{}/thruster_manager/get_config'.format(NS), GetThrusterManagerConfig)
        tm_config = srv()
        
        self.assertEqual(tm_config.tf_prefix, '/test_vehicle/')
        self.assertEqual(tm_config.base_link, 'base_link')
        self.assertEqual(tm_config.thruster_frame_base, 'thruster_')
        self.assertEqual(tm_config.thruster_topic_suffix, '/input')
        self.assertEqual(tm_config.timeout, -1)
        self.assertEqual(tm_config.max_thrust, 1000.0)
        self.assertEqual(tm_config.n_thrusters, 3)

        if axis == 'x':
            tam_flat = AXIS_X_TAM.flatten()
        elif axis == 'y':
            tam_flat = AXIS_Y_TAM.flatten()
        elif axis == 'z':
            tam_flat = AXIS_Z_TAM.flatten()

        self.assertEqual(len(tm_config.allocation_matrix), tam_flat.size)
        for x, y in zip(tam_flat, tm_config.allocation_matrix):
            self.assertAlmostEqual(x, y)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_thruster_allocator', TestThrusterAllocator)
    

