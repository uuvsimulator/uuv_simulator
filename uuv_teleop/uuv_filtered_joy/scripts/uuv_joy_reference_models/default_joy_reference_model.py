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
import numpy as np
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy


class JoyReferenceModel(object):
    """Generation of joystick velocity reference signal."""

    LABEL = 'Joystick reference model'

    def __init__(self):
        """Class constructor."""
        self._namespace = rospy.get_namespace()
        # Gains given to the velocity reference for each DoF
        self._gains = dict(x=0.1, y=0.1, z=0.1, roll=0.1, pitch=0.1, yaw=0.1)
        # The velocity reference from the joystick input
        self._vjs = np.zeros(6)
        # In this case, it will be equal to the joystick input
        self._vel_output = np.zeros(6)

        if rospy.has_param('~gains'):
            gains = rospy.get_param('~gains')
            for tag in gains:
                if tag not in self._gains:
                    raise rospy.ROSException('Invalid gain id, id=' + tag)
                else:
                    self._gains[tag] = gains[tag]

        # Dead zone for the axes
        self._axes_dead_zone = 0.7
        if rospy.has_param('dead_zone'):
            self._axes_dead_zone = rospy.get_param('dead_zone')
            if self._axes_dead_zone < 0:
                raise rospy.ROSException()

        if not rospy.has_param('~joystick_mapping'):
            raise rospy.ROSException('Joystick mapping not available')

        self._joystick_mapping = rospy.get_param('~joystick_mapping')

        # Mapping of the joystick output with each DoF
        if not rospy.has_param('~output_mapping'):
            raise rospy.ROSException('The output pose mapping is not '
                                     'available')

        self._output_mapping = rospy.get_param('~output_mapping')

        # Deadman button to activate this joystick node
        self._deadman_button = None

        if rospy.has_param('~deadman_button'):
            self._deadman_button = str(rospy.get_param('~deadman_button'))
            if self._deadman_button not in self._joystick_mapping['buttons']:
                raise rospy.ROSException('Invalid dead man button')

        # Checking if all the buttons and axes assigned to the DoFs are valid
        for dof in self._gains:
            if dof not in self._output_mapping:
                raise rospy.ROSException('There are DoFs missing mapping in '
                                         'the output mapping table')

        # Removing from the joy input parsing table the buttons and axes
        # that are not used
        self._joy_input = dict()
        for dof in self._output_mapping:
            for type_input in self._output_mapping[dof].keys():
                if type_input not in self._joy_input:
                    self._joy_input[type_input] = dict()
                names = self._output_mapping[dof][type_input]['name']
                for name in names:
                    self._joy_input[type_input][name] = 0

        # A timeout to zero the joystick output if the transmission from the
        # device is inactive, in seconds
        self._timeout = 0.5
        if rospy.has_param('~timeout'):
            self._timeout = rospy.get_param('~timeout')
            if self._timeout <= 0 and self._timeout != -1:
                raise rospy.ROSException('Invalid timeout value')

        # Node update
        self._publish_rate = 25
        if rospy.has_param('~publish_rate'):
            self._publish_rate = rospy.get_param('~publish_rate')
            if self._publish_rate <= 0:
                rospy.ROSException('Invalid publish rate')

        # Stores last joy update to checkout for timeout
        self._last_joy_update = rospy.get_time()
        # Subscribe to the joystick input
        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_callback,
                                         queue_size=1)

        self._command_pub = rospy.Publisher('command', TwistStamped, queue_size=1)

    def _get_button(self, tag):
        assert tag in self._joystick_mapping['buttons'], 'Invalid button tag, tag=' + str(tag)
        return self._joystick_mapping['buttons'][tag]

    def _get_axis(self, tag):
        assert tag in self._joystick_mapping['axes'], 'Invalid axis tag, tag=' + str(tag)
        return self._joystick_mapping['axes'][tag]

    def _reset_joy(self):
        if 'buttons' in self._joy_input:
            for button in self._joy_input['buttons']:
                self._joy_input['buttons'][button] = 0
        if 'axes' in self._joy_input:
            for axis in self._joy_input['axes']:
                self._joy_input['axes'][axis] = 0

    def _parse_joy(self, joy):
        # Update joy update time stamp
        self._last_joy_update = rospy.get_time()
        # Check if the message is empty
        if len(joy.buttons) == 0 or len(joy.axes) == 0:
            rospy.loginfo('Receiving empty Joy message')
            self._reset_joy()
            return
        # If deadman button was given, check if it is activated
        if self._deadman_button is not None:
            deadman_on = joy.buttons[self._get_button(self._deadman_button)]
        else:
            deadman_on = True

        if 'buttons' in self._joy_input:
            for button in self._joy_input['buttons']:
                if not deadman_on:
                    self._joy_input['buttons'][button] = 0
                else:
                    self._joy_input['buttons'][button] = \
                        joy.buttons[self._get_button(button)]
        if 'axes' in self._joy_input:
            for axis in self._joy_input['axes']:
                if not deadman_on:
                    self._joy_input['axes'][axis] = 0
                elif abs(joy.axes[self._get_axis(axis)]) < self._axes_dead_zone:
                    self._joy_input['axes'][axis] = 0
                else:
                    self._joy_input['axes'][axis] = joy.axes[self._get_axis(axis)]

    def _joy_callback(self, msg):
        self._parse_joy(msg)

        vjs = np.zeros(6)
        for i, dof in zip(range(6), ['x', 'y', 'z', 'roll', 'pitch', 'yaw']):
            for b in self._output_mapping[dof]:
                names = self._output_mapping[dof][b]['name']
                gains = self._output_mapping[dof][b]['gain']
                for name, gain in zip(names, gains):
                    vjs[i] += gain * self._joy_input[b][name]
            vjs[i] *= self._gains[dof]

        self._vjs = vjs

    def _update(self):
        self._vel_output = self._vjs

    def _get_vel_output(self):
        return self._vel_output

    def run(self):
        """
        Run update loop to generate the velocity reference from the joystick
        holding the last updated value and considering a timeout to zero the
        reference in case the joystick stays unresponsive
        """
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            # Reset the joystick input if its has been inactive for too long
            if self._timeout != -1:
                if rospy.get_time() - self._last_joy_update > self._timeout:
                    self._reset_joy()
            self._update()
            twist = TwistStamped()
            twist.header.stamp = rospy.Time().now()

            output = self._vel_output

            twist.twist.linear.x = output[0]
            twist.twist.linear.y = output[1]
            twist.twist.linear.z = output[2]

            twist.twist.angular.x = output[3]
            twist.twist.angular.y = output[4]
            twist.twist.angular.z = output[5]

            self._command_pub.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    rospy.loginfo('Starting [joy_reference_model] node, ns=%s' % rospy.get_namespace())

    rospy.init_node('joy_reference_model')
    joy_ref = JoyReferenceModel()
    joy_ref.run()
    rospy.spin()
