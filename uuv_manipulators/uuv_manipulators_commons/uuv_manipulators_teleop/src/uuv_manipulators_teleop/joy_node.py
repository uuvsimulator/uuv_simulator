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

import sys
import rospy
from sensor_msgs.msg import Joy
import PyKDL
import numpy as np


class JoyNode:
    def __init__(self):
        ns = [item for item in rospy.get_namespace().split('/') if len(item) > 0]
        print 'list=', ns
        if len(ns) != 2:
            rospy.ROSException('The controller must be called inside the namespace  the manipulator namespace')

        self._namespace = ns[0]
        self._arm_name = ns[1]

        if not rospy.has_param('~joystick_mapping'):
            raise rospy.ROSException('Joystick mapping not available')

        self._joystick_mapping = rospy.get_param('~joystick_mapping')

        # Parsed joy input
        self._joy_input = dict(buttons=dict(), axes=dict())
        for button in self._joystick_mapping['buttons']:
            self._joy_input['buttons'][button] = 0
        for axis in self._joystick_mapping['axes']:
            if axis not in ['left_trigger', 'right_trigger']:
                self._joy_input['axes'][axis] = 0
            else:
                self._joy_input['axes'][axis] = 1

        self._axes_dead_zone = 0.2

        self._deadman_button = None
        if rospy.has_param('~deadman_button'):
            self._deadman_button = rospy.get_param('~deadman_button')
            if self._deadman_button not in self._joy_input['buttons']:
                raise rospy.ROSException('Invalid deadman button tag')

        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_callback)

    def _is_active(self):
        # Check if deadman button is on
        is_active = False
        if self._deadman_button is None:
            is_active = True
        else:
            is_active = self._joy_input['buttons'][self._deadman_button] == 1
        return is_active

    def _joy_callback(self, joy):
        self._parse_joy(joy)

    def _parse_joy(self, joy):
        self._joy_input = {}
        for key in self._joystick_mapping:
            self._joy_input[key] = {}
            for b in self._joystick_mapping[key]:
                self._joy_input[key][b] = 0
        for button in self._joystick_mapping['buttons']:
            self._joy_input['buttons'][button] = joy.buttons[self._joystick_mapping['buttons'][button]]
        for axis in self._joystick_mapping['axes']:
            self._joy_input['axes'][axis] = joy.axes[self._joystick_mapping['axes'][axis]]
            if axis not in ['left_trigger', 'right_trigger'] and abs(self._joy_input['axes'][axis]) < self._axes_dead_zone:
                self._joy_input['axes'][axis] = 0.0
