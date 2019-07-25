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
import unittest
import numpy as np
import random
from tf_quaternion import transformations
from uuv_thrusters import ThrusterManager
from uuv_thrusters.models import Thruster

PKG = 'uuv_thruster_manager'
roslib.load_manifest(PKG)

IDX = 0
TOPIC = '/thruster'
AXES = [
    np.array([1, 0, 0, 0]),
    np.array([0, 1, 0, 0]),
    np.array([0, 0, 1, 0])
]

def get_force_vector(pos, orientation, axis):
    thrust_body = transformations.quaternion_matrix(orientation).dot(
        axis.transpose())[0:3]
    torque_body = np.cross(pos, thrust_body)
    return np.hstack((thrust_body, torque_body)).transpose()

class TestThrusters(unittest.TestCase):
    def test_thruster(self):
        # Use random positions and quaternions
        for axis in AXES:
            pos = np.random.rand(3)
            q = transformations.random_quaternion()
            
            thruster = Thruster(
                index=IDX,
                topic=TOPIC,
                pos=pos,
                orientation=q,
                axis=axis)

            self.assertEqual(thruster.index, IDX)
            self.assertEqual(thruster.topic, TOPIC)
            self.assertTrue((thruster.tam_column == get_force_vector(pos, q, axis)).all())

    def test_thruster_proportional(self):
        # Use random positions and quaternions
        for axis in AXES:
            pos = np.random.rand(3)
            q = transformations.random_quaternion()
            gain = random.random()
            
            thruster = Thruster.create_thruster(
                'proportional',
                IDX,
                TOPIC,
                pos,
                q,
                axis,
                gain=gain)

            self.assertEqual(thruster.index, IDX)
            self.assertEqual(thruster.topic, TOPIC)
            self.assertTrue((thruster.tam_column == get_force_vector(pos, q, axis)).all())

            self.assertEqual(thruster.get_thrust_value(0), 0)
            self.assertEqual(thruster.get_command_value(0), 0)

            command = np.linspace(-100, 100, 10)
            for x in command:
                y = thruster.get_thrust_value(x)
                self.assertEqual(y, gain * np.abs(x) * x)

            thrust = np.linspace(-50000, 50000, 10)
            for x in thrust:
                y = thruster.get_command_value(x)
                self.assertEqual(y, np.sign(x) * np.sqrt(np.abs(x) / gain))
    
    def test_thruster_custom(self):
        input_values = np.linspace(-50, 50, 10)
        output_values = np.linspace(-10000, 10000, 10)

        gain = 20000.0 / 100.0

        # Use random positions and quaternions
        for axis in AXES:
            pos = np.random.rand(3)
            q = transformations.random_quaternion()
                        
            thruster = Thruster.create_thruster(
                'custom',
                IDX,
                TOPIC,
                pos,
                q,
                axis,
                input=input_values,
                output=output_values)

            self.assertEqual(thruster.index, IDX)
            self.assertEqual(thruster.topic, TOPIC)
            self.assertTrue((thruster.tam_column == get_force_vector(pos, q, axis)).all())

            self.assertTrue(np.isclose(thruster.get_thrust_value(0), 0))
            self.assertTrue(np.isclose(thruster.get_command_value(0), 0))

            x = random.random() * 10
            self.assertTrue(np.isclose(thruster.get_thrust_value(x), gain * x))
            y = random.random() * 10000
            self.assertTrue(np.isclose(thruster.get_command_value(y), y / gain))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_thrusters', TestThrusters)