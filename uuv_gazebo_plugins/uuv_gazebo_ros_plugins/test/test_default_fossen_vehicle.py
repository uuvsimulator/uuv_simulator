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
import rostest
import unittest
from geometry_msgs.msg import Vector3, Inertia
from uuv_gazebo_ros_plugins_msgs.msg import UnderwaterObjectModel
from uuv_gazebo_ros_plugins_msgs.srv import *

PKG = 'uuv_gazebo_ros_plugins'
NAME = 'test_default_fossen_vehicle'

import roslib; roslib.load_manifest(PKG)


class TestDefaultFossenVehicle(unittest.TestCase):
    # def tearDownClass(self):
        # FIXME Temporary solution to avoid gzserver lingering after the
        # simulation node is killed (Gazebo 9.1)
        # os.system('killall -9 gzserver')

    def test_get_model_parameters(self):
        rospy.wait_for_service('/vehicle/get_model_properties')

        s = rospy.ServiceProxy('/vehicle/get_model_properties',
                               GetModelProperties)
        models = s()

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
        # Generate index numbers for the diagonal elements of the matrices
        d_idxs = [i*6 + j for i, j in zip(range(6), range(6))]
        self.assertEqual(len(models.models[0].added_mass), 36)
        for i in range(len(models.models[0].added_mass)):
            if i in d_idxs:
                self.assertEqual(models.models[0].added_mass[i], 1.0)
            else:
                self.assertEqual(models.models[0].added_mass[i], 0.0)

        self.assertEqual(len(models.models[0].linear_damping), 36)
        for i in range(len(models.models[0].linear_damping)):
            if i in d_idxs:
                self.assertEqual(models.models[0].linear_damping[i], 1.0)
            else:
                self.assertEqual(models.models[0].linear_damping[i], 0.0)

        self.assertEqual(len(models.models[0].linear_damping_forward_speed), 36)
        for i in range(len(models.models[0].linear_damping_forward_speed)):
            if i in d_idxs:
                self.assertEqual(models.models[0].linear_damping_forward_speed[i], 1.0)
            else:
                self.assertEqual(models.models[0].linear_damping_forward_speed[i], 0.0)

        self.assertEqual(len(models.models[0].quadratic_damping), 36)
        for i in range(len(models.models[0].quadratic_damping)):
            if i in d_idxs:
                self.assertEqual(models.models[0].quadratic_damping[i], 1.0)
            else:
                self.assertEqual(models.models[0].quadratic_damping[i], 0.0)

        # Tests if some of the parameters match to the ones given in the URDF
        self.assertEqual(models.models[0].fluid_density, 1028.0)
        self.assertEqual(models.models[0].volume, 1.0)
        self.assertEqual(models.models[0].bbox_height, 1.0)
        self.assertEqual(models.models[0].bbox_length, 1.0)
        self.assertEqual(models.models[0].bbox_width, 1.0)

    def test_set_fluid_density(self):
        rospy.wait_for_service('/vehicle/set_fluid_density')
        set_func = rospy.ServiceProxy('/vehicle/set_fluid_density', SetFloat)

        rospy.wait_for_service('/vehicle/get_fluid_density')
        get_func = rospy.ServiceProxy('/vehicle/get_fluid_density', GetFloat)

        self.assertEqual(get_func().data, 1028.0)
        self.assertTrue(set_func(1025.0).success)
        self.assertEqual(get_func().data, 1025.0)
        self.assertTrue(set_func(1028.0).success)

    def test_volume_offset(self):
        rospy.wait_for_service('/vehicle/set_volume_offset')
        set_func = rospy.ServiceProxy('/vehicle/set_volume_offset', SetFloat)

        rospy.wait_for_service('/vehicle/get_volume_offset')
        get_func = rospy.ServiceProxy('/vehicle/get_volume_offset', GetFloat)

        rospy.wait_for_service('/vehicle/get_model_properties')
        get_model = rospy.ServiceProxy('/vehicle/get_model_properties',
                                       GetModelProperties)

        # Test that offset has changed
        self.assertEqual(get_func().data, 0.0)
        self.assertTrue(set_func(1.0).success)
        self.assertEqual(get_func().data, 1.0)

        # Test that the actual volume has NOT changed, offset should only
        # be used during the computation of forces
        self.assertEqual(get_model().models[0].volume, 1.0)

        self.assertTrue(set_func(0.0).success)

    def test_added_mass_scaling(self):
        rospy.wait_for_service('/vehicle/set_added_mass_scaling')
        set_func = rospy.ServiceProxy('/vehicle/set_added_mass_scaling', SetFloat)

        rospy.wait_for_service('/vehicle/get_added_mass_scaling')
        get_func = rospy.ServiceProxy('/vehicle/get_added_mass_scaling', GetFloat)

        self.assertEqual(get_func().data, 1.0)
        self.assertTrue(set_func(0.8).success)
        self.assertEqual(get_func().data, 0.8)
        self.assertTrue(set_func(1.0).success)

    def test_damping_scaling(self):
        rospy.wait_for_service('/vehicle/set_damping_scaling')
        set_func = rospy.ServiceProxy('/vehicle/set_damping_scaling', SetFloat)

        rospy.wait_for_service('/vehicle/get_damping_scaling')
        get_func = rospy.ServiceProxy('/vehicle/get_damping_scaling', GetFloat)

        self.assertEqual(get_func().data, 1.0)
        self.assertTrue(set_func(0.8).success)
        self.assertEqual(get_func().data, 0.8)
        self.assertTrue(set_func(1.0).success)

    def test_volume_scaling(self):
        rospy.wait_for_service('/vehicle/set_volume_scaling')
        set_func = rospy.ServiceProxy('/vehicle/set_volume_scaling', SetFloat)

        rospy.wait_for_service('/vehicle/get_volume_scaling')
        get_func = rospy.ServiceProxy('/vehicle/get_volume_scaling', GetFloat)

        self.assertEqual(get_func().data, 1.0)
        self.assertTrue(set_func(0.8).success)
        self.assertEqual(get_func().data, 0.8)
        self.assertTrue(set_func(1.0).success)

    def test_added_mass_offset(self):
        rospy.wait_for_service('/vehicle/set_added_mass_offset')
        set_func = rospy.ServiceProxy('/vehicle/set_added_mass_offset', SetFloat)

        rospy.wait_for_service('/vehicle/get_added_mass_offset')
        get_func = rospy.ServiceProxy('/vehicle/get_added_mass_offset', GetFloat)

        self.assertEqual(get_func().data, 0.0)

        self.assertTrue(set_func(1.0).success)
        self.assertEqual(get_func().data, 1.0)
        self.assertTrue(set_func(0.0).success)

    def test_linear_damping_offset(self):
        rospy.wait_for_service('/vehicle/set_linear_damping_offset')
        set_func = rospy.ServiceProxy('/vehicle/set_linear_damping_offset', SetFloat)

        rospy.wait_for_service('/vehicle/get_linear_damping_offset')
        get_func = rospy.ServiceProxy('/vehicle/get_linear_damping_offset', GetFloat)

        self.assertEqual(get_func().data, 0.0)
        self.assertTrue(set_func(1.0).success)
        self.assertEqual(get_func().data, 1.0)
        self.assertTrue(set_func(0.0).success)

    def test_linear_forward_speed_damping_offset(self):
        rospy.wait_for_service('/vehicle/set_linear_forward_speed_damping_offset')
        set_func = rospy.ServiceProxy('/vehicle/set_linear_forward_speed_damping_offset', SetFloat)

        rospy.wait_for_service('/vehicle/get_linear_forward_speed_damping_offset')
        get_func = rospy.ServiceProxy('/vehicle/get_linear_forward_speed_damping_offset', GetFloat)

        self.assertEqual(get_func().data, 0.0)
        self.assertTrue(set_func(1.0).success)
        self.assertEqual(get_func().data, 1.0)
        self.assertTrue(set_func(0.0).success)

    def test_linear_forward_speed_damping_offset(self):
        rospy.wait_for_service('/vehicle/set_nonlinear_damping_offset')
        set_func = rospy.ServiceProxy('/vehicle/set_nonlinear_damping_offset', SetFloat)

        rospy.wait_for_service('/vehicle/get_nonlinear_damping_offset')
        get_func = rospy.ServiceProxy('/vehicle/get_nonlinear_damping_offset', GetFloat)

        self.assertEqual(get_func().data, 0.0)
        self.assertTrue(set_func(1.0).success)
        self.assertEqual(get_func().data, 1.0)
        self.assertTrue(set_func(0.0).success)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestDefaultFossenVehicle)
