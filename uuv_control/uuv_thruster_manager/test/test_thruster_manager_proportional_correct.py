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
import unittest
import numpy as np
from uuv_thrusters import ThrusterManager

PKG = 'uuv_thruster_manager'
roslib.load_manifest(PKG)

rospy.init_node('test_thruster_manager_proportional_correct', anonymous=True)

MANAGER = ThrusterManager()

REFERENCE_TAM = np.array([
    [1, 0, 0, 0, 0, 0],
    [0.87758256, 0, -0.47942554, 0.47942554, 0.47942554, 0.87758256],
    [0.87758256, 0.47942554, 0, -0.47942554, 0.87758256, -0.87758256]
]).T

class TestThrusterManagerProportionalCorrect(unittest.TestCase):
    def test_initialization(self):        
        self.assertEqual(MANAGER.namespace, '/test_vehicle/')

        self.assertEqual(MANAGER.config['tf_prefix'], 'test_vehicle')
        self.assertEqual(MANAGER.config['base_link'], 'base_link')
        self.assertEqual(MANAGER.config['thruster_topic_prefix'], 'thrusters/')        
        self.assertEqual(MANAGER.config['thruster_frame_base'], 'thruster_')
        self.assertEqual(MANAGER.config['thruster_topic_suffix'], '/input')
        self.assertEqual(MANAGER.config['timeout'], -1)
        self.assertEqual(MANAGER.config['max_thrust'], 1000.0)
        self.assertEqual(MANAGER.config['update_rate'], 50)

        self.assertEqual(MANAGER.n_thrusters, 3)

        self.assertEqual(REFERENCE_TAM.shape, MANAGER.configuration_matrix.shape)        

        self.assertTrue(np.isclose(REFERENCE_TAM, MANAGER.configuration_matrix).all())

    def test_thrusters(self):
        self.assertEqual(len(MANAGER.thrusters), 3)

        for i in range(len(MANAGER.thrusters)):
            self.assertEqual(MANAGER.thrusters[i].index, i)
            self.assertEqual(MANAGER.thrusters[i].topic, 'thrusters/{}/input'.format(i))
            self.assertEqual(MANAGER.thrusters[i].LABEL, 'proportional')
            self.assertTrue(np.isclose(REFERENCE_TAM[:, i].flatten(), MANAGER.thrusters[i].tam_column).all())

    def test_processing_gen_forces(self):
        for _ in range(10):
            gen_force = np.random.rand(6) * 100
            thrust_forces = MANAGER.compute_thruster_forces(gen_force)

            ref_thrust_forces = np.linalg.pinv(REFERENCE_TAM).dot(gen_force)

            self.assertTrue(np.isclose(ref_thrust_forces, thrust_forces).all())

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_thruster_manager_proportional_correct', TestThrusterManagerProportionalCorrect)