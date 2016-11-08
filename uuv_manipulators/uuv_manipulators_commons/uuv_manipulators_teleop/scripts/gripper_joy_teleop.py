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
from uuv_manipulators_msgs.msg import EndeffectorCommand
from uuv_manipulators_teleop import JoyNode


class GripperTeleopNode(JoyNode):
    def __init__(self):
        JoyNode.__init__(self)
        # Initial values for the gripper joystick axes
        self._joy_input['axes']['left_trigger'] = 1.0
        self._joy_input['axes']['right_trigger'] = 1.0

        self._gripper_command_pub = rospy.Publisher(
            'gripper_control/command', EndeffectorCommand, queue_size=10)
        self._command = EndeffectorCommand()
        self._command.command = EndeffectorCommand.EE_STOP
        self._command.ratio = 0

        self._update_rate = 1000
        if rospy.has_param('~update_rate'):
            self._update_rate = rospy.get_param('~update_rate')

        rate = rospy.Rate(self._update_rate)
        while not rospy.is_shutdown():
            self._update()
            self._gripper_command_pub.publish(self._command)
            rate.sleep()

    def _update(self):
        if self._is_active():
            if 'left_trigger' not in self._joy_input['axes'] or 'right_trigger' not in self._joy_input['axes']:
                self._command.command = EndeffectorCommand.EE_STOP
                self._command.ratio = 0
                return
            close_gripper = -1 * (self._joy_input['axes']['left_trigger'] / 2.0 - 0.5)
            open_gripper = -1 * (self._joy_input['axes']['right_trigger'] / 2.0 - 0.5)

            self._command.command = EndeffectorCommand.EE_MOVE
            self._command.ratio = open_gripper - close_gripper
        else:
            self._command.command = EndeffectorCommand.EE_STOP
            self._command.ratio = 0

if __name__ == "__main__":
    print 'Starting Gripper Teleop'
    rospy.init_node('gripper_teleop');
    try:
        node = GripperTeleopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print 'GripperTeleopNode::Exception'
    print 'Leaving GripperTeleopNode'
