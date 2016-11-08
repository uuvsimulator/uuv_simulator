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
from uuv_manipulator_interfaces import GripperInterface
from uuv_manipulators_msgs.msg import EndeffectorCommand
from sensor_msgs.msg import Joy

class GripperController(object):
    COMMAND = ['move', 'stop']

    def __init__(self):
        # Timeout for input inactivity
        self._timeout = 0.01

        # Last input update
        self._last_update = rospy.get_time()

        # Initializing the gripper interface for the current namespace
        self._gripper = GripperInterface()

        # Retrieve the publish rate
        self._publish_rate = 1000
        if rospy.has_param('~publish_rate'):
            self._publish_rate = rospy.get_param('~publish_rate')

        if self._publish_rate <= 0:
            raise rospy.ROSException('Invalid negative publish rate')

        self._command_sub = rospy.Subscriber('gripper_control/command',
            EndeffectorCommand,
            self._on_gripper_command)

        self._command = 'stop'

        self._pos_goal = self._gripper.closed_pos
        self._ratio_goal = self._gripper.get_position_ratio(self._gripper.closed_pos)

        print 'ratio=', self._ratio_goal

        self._kp = 100

        if rospy.has_param('~kp'):
            self._kp = rospy.get_param('~kp')

        if self._gripper.control_joint is None:
            raise rospy.ROSException('Gripper cannot be controlled')

        self._limits = self._gripper.control_joint_limits

        self._controller_update_timer = rospy.Timer(
            rospy.Duration(1.0 / self._publish_rate), self._update)

        rospy.on_shutdown(self._on_shutdown)

    def _on_shutdown(self):
        self._controller_update_timer.shutdown()

    def _on_gripper_command(self, command):
        if not self._gripper.is_ready:
            return
        self._command = command.command

        if self._command == 'move':
            self._ratio_goal = command.ratio + self._gripper.control_joint_pos_ratio
        elif self._command == 'stop':
            self._ratio_goal = self._gripper.control_joint_pos_ratio

        if self._ratio_goal < 0:
            self._ratio_goal = 0.0
        elif self._ratio_goal > 1:
            self._ratio_goal = 1.0

        self._last_update = rospy.get_time()

    def _reset_gripper_command(self):
        self._command = 'stop'

    def _update(self, event):
        if rospy.get_time() - self._last_update > self._timeout:
            self._reset_gripper_command()
        self._pos_goal = self._ratio_goal * (self._gripper.fully_open_pos - self._gripper.closed_pos) + self._gripper.closed_pos
        error = self._pos_goal - self._gripper.control_joint_position
        torque = self._kp * error
        self._gripper.set_command(torque)

if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0] + '_gripper'
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    jt_controller = GripperController()

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
