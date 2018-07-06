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
from __future__ import print_function
import rospy
import unittest
import numpy as np
from geometry_msgs.msg import Vector3, Inertia
from uuv_gazebo_ros_plugins_msgs.msg import UnderwaterObjectModel
from uuv_gazebo_ros_plugins_msgs.srv import *

PKG = 'uuv_gazebo_ros_plugins'
NAME = 'test_sphere_vehicle'

import roslib; roslib.load_manifest(PKG)

# Sphere model radius
RADIUS = 0.1
# Damping coefficient
CD = 0.5


class TestSphereVehicle(unittest.TestCase):
    # def tearDownClass(self):
        # FIXME Temporary solution to avoid gzserver lingering after the
        # simulation node is killed (Gazebo 9.1)
        # os.system('killall -9 gzserver')

    def test_get_model_parameters(self):
        rospy.wait_for_service('/vehicle/get_model_properties')

        get_models = rospy.ServiceProxy('/vehicle/get_model_properties',
                                        GetModelProperties)
        models = get_models()

        self.assertEqual(len(models.link_names), 1)
        self.assertEqual(len(models.models), 1)

        # Test the name of the link
        self.assertEqual(
            models.link_names[0], 'vehicle/base_link',
            'Link name is invalid, name=' + str(models.link_names[0]))

        # Test message types
        self.assertIsInstance(models.models[0].added_mass, tuple)
        self.assertIsInstance(models.models[0].linear_damping, tuple)
        self.assertIsInstance(models.models[0].linear_damping_forward_speed, tuple)
        self.assertIsInstance(models.models[0].quadratic_damping, tuple)
        self.assertIsInstance(models.models[0].volume, float)
        self.assertIsInstance(models.models[0].bbox_length, float)
        self.assertIsInstance(models.models[0].bbox_width, float)
        self.assertIsInstance(models.models[0].bbox_height, float)
        self.assertIsInstance(models.models[0].fluid_density, float)
        self.assertIsInstance(models.models[0].neutrally_buoyant, bool)
        self.assertIsInstance(models.models[0].cob, Vector3)
        self.assertIsInstance(models.models[0].inertia, Inertia)

        # Test size of the parameter lists
        self.assertEqual(len(models.models[0].added_mass), 36)
        self.assertEqual(len(models.models[0].linear_damping), 36)
        self.assertEqual(len(models.models[0].linear_damping_forward_speed), 36)
        self.assertEqual(len(models.models[0].quadratic_damping), 36)

        # Tests if some of the parameters match to the ones given in the URDF
        self.assertEqual(models.models[0].fluid_density, 1028.0)
        self.assertEqual(models.models[0].volume, 0.009727626)
        self.assertEqual(models.models[0].bbox_height, 1.0)
        self.assertEqual(models.models[0].bbox_length, 1.0)
        self.assertEqual(models.models[0].bbox_width, 1.0)

    def test_added_mass_coefs(self):
        rospy.wait_for_service('/vehicle/get_model_properties')

        get_models = rospy.ServiceProxy('/vehicle/get_model_properties',
                                        GetModelProperties)
        models = get_models()

        d_idxs = [i*6 + j for i, j in zip(range(3), range(3))]
        sphere_ma = 2.0 / 3.0 * models.models[0].fluid_density * np.pi * \
            RADIUS**3.0
        for i in range(len(models.models[0].added_mass)):
            if i in d_idxs:
                self.assertLess(abs(models.models[0].added_mass[i] - sphere_ma), 0.001)
            else:
                self.assertEqual(models.models[0].added_mass[i], 0.0)

    def test_nonlinear_damping_coefs(self):
        rospy.wait_for_service('/vehicle/get_model_properties')

        get_models = rospy.ServiceProxy('/vehicle/get_model_properties',
                                        GetModelProperties)
        models = get_models()

        area_section = np.pi * RADIUS**2
        dq = -0.5 * models.models[0].fluid_density * CD * area_section

        d_idxs = [i*6 + j for i, j in zip(range(3), range(3))]
        for i in range(len(models.models[0].quadratic_damping)):
            if i in d_idxs:
                self.assertLess(abs(models.models[0].quadratic_damping[i] - dq), 0.001)
            else:
                self.assertEqual(models.models[0].quadratic_damping[i], 0.0)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestSphereVehicle)
