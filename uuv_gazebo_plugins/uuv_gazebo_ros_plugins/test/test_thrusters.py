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
import rospy
import unittest
import sys
from time import sleep
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_gazebo_ros_plugins_msgs.srv import GetThrusterConversionFcn, \
    SetThrusterState, GetThrusterState, SetThrusterEfficiency, \
    GetThrusterEfficiency

PKG = 'uuv_gazebo_ros_plugins'
NAME = 'test_thrusters'

import roslib; roslib.load_manifest(PKG)


class TestThrusters(unittest.TestCase):
    def __init__(self, *args):
        super(TestThrusters, self).__init__(*args)
        rospy.init_node('test_thrusters', anonymous=True)

        self.thruster_input_pub = dict()
        for i in range(3):
            self.thruster_input_pub[i] = rospy.Publisher(
                '/vehicle/thrusters/%d/input' % i, FloatStamped, queue_size=1)

    def test_input_output_topics_exist(self):
        pub = rospy.Publisher('/vehicle/thrusters/0/input', FloatStamped,
                              queue_size=1)

        for k in self.thruster_input_pub:
            # Publishing set point to rotor velocity
            input_message = FloatStamped()
            input_message.header.stamp = rospy.Time.now()
            input_message.data = 0.2

            self.thruster_input_pub[k].publish(input_message)
            sleep(1)

            output = rospy.wait_for_message('/vehicle/thrusters/%d/thrust' % k,
                                            FloatStamped, timeout=30)
            self.assertIsNot(output.data, 0.0)

            # Turning thruster off
            input_message.data = 0.0
            pub.publish(input_message)

    def test_convertion_fcn_parameters(self):
        # Testing thruster #0 - basic/proportional model
        rospy.wait_for_service(
            '/vehicle/thrusters/0/get_thruster_conversion_fcn')

        get_thruster_convertion_fcn = rospy.ServiceProxy(
            '/vehicle/thrusters/0/get_thruster_conversion_fcn',
            GetThrusterConversionFcn)

        fcn = get_thruster_convertion_fcn()

        self.assertEqual(fcn.fcn.function_name, 'Basic')
        self.assertEqual(len(fcn.fcn.tags), len(fcn.fcn.data))
        self.assertEqual(len(fcn.fcn.tags), 1)
        self.assertIn('rotor_constant', fcn.fcn.tags)
        self.assertEqual(fcn.fcn.data[0], 0.001)

        # Testing thruster #1 - Bessa/nonlinear model
        rospy.wait_for_service(
            '/vehicle/thrusters/1/get_thruster_conversion_fcn')

        get_thruster_convertion_fcn = rospy.ServiceProxy(
            '/vehicle/thrusters/1/get_thruster_conversion_fcn',
            GetThrusterConversionFcn)

        fcn = get_thruster_convertion_fcn()

        bessa_tags = ['rotor_constant_l', 'rotor_constant_r', 'delta_l',
                      'delta_r']
        bessa_params = [0.001, 0.001, -0.01, 0.01]
        self.assertEqual(fcn.fcn.function_name, 'Bessa')
        self.assertEqual(len(fcn.fcn.tags), len(fcn.fcn.data))
        self.assertEqual(len(fcn.fcn.tags), 4)
        for t, p in zip(fcn.fcn.tags, fcn.fcn.data):
            self.assertIn(t, bessa_tags)
            self.assertEqual(p, bessa_params[bessa_tags.index(t)])

        # Testing thruster #2 - Linear interpolation
        rospy.wait_for_service(
            '/vehicle/thrusters/2/get_thruster_conversion_fcn')

        get_thruster_convertion_fcn = rospy.ServiceProxy(
            '/vehicle/thrusters/2/get_thruster_conversion_fcn',
            GetThrusterConversionFcn)

        fcn = get_thruster_convertion_fcn()

        self.assertEqual(fcn.fcn.function_name, 'LinearInterp')
        self.assertEqual(len(fcn.fcn.tags), len(fcn.fcn.data))
        self.assertEqual(len(fcn.fcn.tags), 0)
        self.assertEqual(len(fcn.fcn.lookup_table_input),
                         len(fcn.fcn.lookup_table_output))
        self.assertListEqual([-0.1, 0.0, 0.1],
                             list(fcn.fcn.lookup_table_input))
        self.assertListEqual([-0.01, 0.0, 0.01],
                             list(fcn.fcn.lookup_table_output))

    def test_change_thruster_state(self):
        for i in range(3):
            rospy.wait_for_service(
                '/vehicle/thrusters/%d/set_thruster_state' % i)
            set_state = rospy.ServiceProxy(
                '/vehicle/thrusters/%d/set_thruster_state' % i,
                SetThrusterState)
            self.assertTrue(set_state(False).success)

            # Test that thruster is off
            rospy.wait_for_service(
                '/vehicle/thrusters/%d/get_thruster_state' % i)
            get_state = rospy.ServiceProxy(
                '/vehicle/thrusters/%d/get_thruster_state' % i,
                GetThrusterState)

            self.assertFalse(get_state().is_on)

            # Turn thruster on again
            self.assertTrue(set_state(True).success)
            self.assertTrue(get_state().is_on)

    def test_change_thrust_efficiency(self):
        for i in range(3):
            rospy.wait_for_service(
                '/vehicle/thrusters/%d/set_thrust_force_efficiency' % i)
            set_efficiency = rospy.ServiceProxy(
                '/vehicle/thrusters/%d/set_thrust_force_efficiency' % i,
                SetThrusterEfficiency)
            self.assertTrue(set_efficiency(0.5).success)

            # Test that thruster is off
            rospy.wait_for_service(
                '/vehicle/thrusters/%d/get_thrust_force_efficiency' % i)
            get_efficiency = rospy.ServiceProxy(
                '/vehicle/thrusters/%d/get_thrust_force_efficiency' % i,
                GetThrusterEfficiency)

            self.assertEqual(get_efficiency().efficiency, 0.5)

            # Turn thruster on again
            self.assertTrue(set_efficiency(1.0).success)
            self.assertEqual(get_efficiency().efficiency, 1.0)

    def test_change_dyn_state_efficiency(self):
        for i in range(3):
            rospy.wait_for_service(
                '/vehicle/thrusters/%d/set_dynamic_state_efficiency' % i)
            set_efficiency = rospy.ServiceProxy(
                '/vehicle/thrusters/%d/set_dynamic_state_efficiency' % i,
                SetThrusterEfficiency)
            self.assertTrue(set_efficiency(0.5).success)

            # Test that thruster is off
            rospy.wait_for_service(
                '/vehicle/thrusters/%d/get_dynamic_state_efficiency' % i)
            get_efficiency = rospy.ServiceProxy(
                '/vehicle/thrusters/%d/get_dynamic_state_efficiency' % i,
                GetThrusterEfficiency)

            self.assertEqual(get_efficiency().efficiency, 0.5)

            # Turn thruster on again
            self.assertTrue(set_efficiency(1.0).success)
            self.assertEqual(get_efficiency().efficiency, 1.0)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestThrusters, sys.argv)
