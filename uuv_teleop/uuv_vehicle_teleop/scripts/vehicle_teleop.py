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
import os
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Accel, Vector3
from sensor_msgs.msg import Joy


class VehicleTeleop:
    def __init__(self):
        # Load the mapping for each input
        self._axes = dict(x=4, y=3, z=1, yaw=0)
        # Load the gain for each joystick axis input
        # (default values for the XBox 360 controller)
        self._axes_gain = dict(x=3, y=3, z=0.5, yaw=0.5)
        if rospy.has_param('~mapping'):
            mapping = rospy.get_param('~mapping')
            for tag in self._axes:
                if tag not in mapping:
                    raise rospy.ROSException('Invalid tag in axes mapping, '
                                             'tag=%s' % tag)
                else:
                    if 'axis' in mapping[tag]:
                        self._axes[tag] = mapping[tag]['axis']
                    if 'gain' in mapping[tag]:
                        self._axes_gain[tag] = mapping[tag]['gain']

        # Default for the RB button of the XBox 360 controller
        self._deadman_button = -1
        if rospy.has_param('~deadman_button'):
            self._deadman_button = int(rospy.get_param('~deadman_button'))

        # If these buttons are pressed, the arm will not move
        if rospy.has_param('~exclusion_buttons'):
            self._exclusion_buttons = rospy.get_param('~exclusion_buttons')
            if type(self._exclusion_buttons) in [float, int]:
                self._exclusion_buttons = [int(self._exclusion_buttons)]
            elif type(self._exclusion_buttons) == list:
                for n in self._exclusion_buttons:
                    if type(n) not in [float, int]:
                        raise rospy.ROSException(
                            'Exclusion buttons must be an integer index to the joystick button')
        else:
            self._exclusion_buttons = list()

        self._msg_type = 'twist'
        if rospy.has_param('~type'):
            self._msg_type = rospy.get_param('~type')
            if self._msg_type not in ['twist', 'accel']:
                raise rospy.ROSException('Vehicle teleop output must be either'
                                         ' twist or accel')

        if self._msg_type == 'twist':
            self._output_pub = rospy.Publisher('output', Twist, queue_size=1)
        else:
            self._output_pub = rospy.Publisher('output', Accel, queue_size=1)

        # Joystick topic subscriber
        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_callback)

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()

    def _parse_joy(self, joy=None):
        if self._msg_type == 'twist':
            cmd = Twist()
        else:
            cmd = Accel()
        if joy is not None:
            cmd.linear = Vector3(self._axes_gain['x'] * (joy.axes[self._axes['x']] if abs(joy.axes[self._axes['x']]) > 0.5 else 0.0),
                                 self._axes_gain['y'] * (joy.axes[self._axes['y']] if abs(joy.axes[self._axes['y']]) > 0.5 else 0.0),
                                 self._axes_gain['z'] * (joy.axes[self._axes['z']] if abs(joy.axes[self._axes['z']]) > 0.5 else 0.0))
            cmd.angular = Vector3(0, 0,
                                  self._axes_gain['yaw'] * (joy.axes[self._axes['yaw']] if abs(joy.axes[self._axes['yaw']]) > 0.5 else 0.0))
        else:
            cmd.linear = Vector3(0, 0, 0)
            cmd.angular = Vector3(0, 0, 0)
        return cmd

    def _joy_callback(self, joy):
        # If any exclusion buttons are pressed, do nothing
        for n in self._exclusion_buttons:
            if joy.buttons[n] == 1:
                cmd = self._parse_joy()
                self._output_pub.publish(cmd)
                return

        if self._deadman_button != -1:
            if joy.buttons[self._deadman_button] == 1:
                cmd = self._parse_joy(joy)
            else:
                cmd = self._parse_joy()
        else:
            cmd = self._parse_joy(joy)
        self._output_pub.publish(cmd)

if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    teleop = VehicleTeleop()

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
