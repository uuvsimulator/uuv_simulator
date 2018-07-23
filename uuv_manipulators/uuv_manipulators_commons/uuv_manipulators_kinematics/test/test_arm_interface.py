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
PKG = 'uuv_manipulators_kinematics'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest
import numpy as np
from uuv_manipulator_interfaces import ArmInterface


class TestArmInterface(unittest.TestCase):
    def test_init_interface(self):
         arm = ArmInterface()
         # Test if the namespace and arm name are correct
         self.assertEquals(arm.namespace, '/rexrov/', 'Invalid robot namespace')
         self.assertEquals(arm.arm_name, 'oberon7', 'Invalid arm name')
         self.assertEquals(arm.base_link, 'oberon7/base', 'Invalid manipulator base name')
         self.assertEquals(arm.tip_link, 'oberon7/end_effector', 'Invalid end-effector link name')
         self.assertNotEquals(len(arm.joint_names), 0, 'The list of joint names is empty')
         self.assertEquals(arm.n_links, 6, 'Invalid number of links, n_links=' + str(arm.n_links))

         for name in arm.joint_names:
             self.assertIn(name, arm.joint_angles, 'Joint name %s not listed in the joint positions dictionary' % name)
             self.assertIn(name, arm.joint_velocities, 'Joint name %s not listed in the joint velocities dictionary' % name)
             self.assertIn(name, arm.joint_efforts, 'Joint name %s not listed in the joint efforts dictionary' % name)

    def test_joints_to_kdl(self):
        arm = ArmInterface()
        for idx, name in zip(range(len(arm.joint_names)), arm.joint_names):
            for t in ['positions', 'torques']:
                jnt_array = arm.joints_to_kdl(t, last_joint=name)
                self.assertEquals(jnt_array.rows(), idx + 1,
                                  'Invalid number of joints, joint_idx=%d, last_joint=%s, n_joints=%d' % (idx, name, jnt_array.rows()))

    def test_jacobian(self):
        arm = ArmInterface()
        jac = arm.jacobian()
        self.assertIsNotNone(jac, 'Jacobian matrix is invalid')
        self.assertEquals(jac.shape, (arm.n_links, 6), 'The full Jacobian matrix has the wrong size')

        for idx, name in zip(range(len(arm.link_names)), arm.link_names):
            self.assertEquals(arm.jacobian(end_link=name).shape, (arm.n_links, 6))
            self.assertEquals(arm.jacobian_transpose(end_link=name).shape, (6, arm.n_links))

    def test_home_config(self):
        arm = ArmInterface()
        self.assertIsNotNone(arm.home, 'Home configuration is invalid')


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_arm_interface', TestArmInterface)
