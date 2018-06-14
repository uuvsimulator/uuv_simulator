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
from copy import deepcopy
from uuv_manipulator_interfaces import ArmInterface
from PID import PIDRegulator
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class JointPositionController:
    def __init__(self):
        # Initializing arm interface
        self._arm_interface = ArmInterface()

        # PID controllers
        self._controllers = dict()
        # Current reference for each joint
        self._reference_pos = dict()
        # Output command topics
        self._command_topics = dict()
        # Axes mapping
        self._axes = dict()
        # Axes gain values
        self._axes_gain = dict()

        # Default for the RB button of the XBox 360 controller
        self._deadman_button = 5
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
                        raise rospy.ROSException('Exclusion buttons must be an'
                                                 ' integer index to the joystick button')
        else:
            self._exclusion_buttons = list()

        # Default for the start button of the XBox 360 controller
        self._home_button = 7
        if rospy.has_param('~home_button'):
            self._home_button = int(rospy.get_param('~home_button'))

        # Last joystick update timestamp
        self._last_joy_update = rospy.get_time()
        # Joystick topic subscriber
        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_callback)

        # Reading the controller configuration
        controller_config = rospy.get_param('~controller_config')
        for joint in self._arm_interface.joint_names:
            for tag in controller_config:
                if tag in joint:
                    try:
                        # Read the controller parameters
                        self._controllers[joint] = PIDRegulator(controller_config[tag]['controller']['p'],
                                                                controller_config[tag]['controller']['i'],
                                                                controller_config[tag]['controller']['d'],
                                                                1000)
                        self._command_topics[joint] = rospy.Publisher(
                            controller_config[tag]['topic'],
                            Float64,
                            queue_size=1)
                        self._axes[joint] = controller_config[tag]['joint_input_axis']
                        self._axes_gain[joint] = controller_config[tag]['axis_gain']

                        # Setting the starting reference to the home position
                        # in the robot parameters file
                        self._reference_pos[joint] = deepcopy(self._arm_interface.home[joint])
                    except:
                        raise rospy.ROSException('Error while trying to setup controller for joint <%s>' % joint)

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            pos = self._arm_interface.joint_angles
            for joint in pos:
                u = self._controllers[joint].regulate(self._reference_pos[joint] - pos[joint], rospy.get_time())
                self._command_topics[joint].publish(Float64(u))
            rate.sleep()

    def _check_joint_limits(self, value, joint):
        """Check the joint position and maintain it within joint limits"""
        output = value
        if self._arm_interface.joint_limits[joint] is not None:
            if output < self._arm_interface.joint_limits[joint]['lower']:
                output = self._arm_interface.joint_limits[joint]['lower']
            if output > self._arm_interface.joint_limits[joint]['upper']:
                output = self._arm_interface.joint_limits[joint]['upper']
        return output

    def _joy_callback(self, joy):
        try:
            # If deadman button is not pressed, do nothing
            if not joy.buttons[self._deadman_button] and self._deadman_button != -1:
                return
            # If any exclusion buttons are pressed, do nothing
            for n in self._exclusion_buttons:
                if joy.buttons[n] == 1:
                    return
            if joy.buttons[self._home_button]:
                self._reference_pos = deepcopy(self._arm_interface.home)
                self._last_joy_update = rospy.get_time()
            else:
                # Parse the joystick input to set the joint angle reference
                for joint in self._arm_interface.joint_names:
                    if abs(joy.axes[self._axes[joint]]) < 0.8:
                        continue
                    else:
                        self._reference_pos[joint] += self._axes_gain[joint] * \
                            joy.axes[self._axes[joint]]
                        # Check for the joint limits
                        self._reference_pos[joint] = self._check_joint_limits(self._reference_pos[joint], joint)
                self._last_joy_update = rospy.get_time()
        except Exception, e:
            print 'Error during joy parsing, message=', e

if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    controller = JointPositionController()

    rospy.spin()
    rospy.loginfo('Shuting down [%s] node' % node_name)
