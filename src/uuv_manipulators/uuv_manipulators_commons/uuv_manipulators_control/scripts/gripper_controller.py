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
from PID import PIDRegulator


class GripperController:

    def __init__(self):
        # Timeout for input inactivity
        self._timeout = 0.01

        # Initializing the gripper interface for the current namespace
        self._gripper = GripperInterface()

        # Initial values for the gripper joystick buttons
        # Default - B button from XBox 360 controller
        self._open_button = 1
        if rospy.has_param('~open_button'):
            self._open_button = rospy.get_param('~open_button')

        # Default - X button from XBox 360 controller
        self._close_button = 2
        if rospy.has_param('~close_button'):
            self._close_button = rospy.get_param('~close_button')

        # Default - RB button from XBox 360 controller
        self._deadman_button = 5
        if rospy.has_param('~deadman_button'):
            self._deadman_button = rospy.get_param('~deadman_button')

        if rospy.has_param('~exclusion_buttons'):
            self._exclusion_buttons = rospy.get_param('~exclusion_buttons')
            if type(self._exclusion_buttons) in [float, int]:
                self._exclusion_buttons = [int(self._exclusion_buttons)]
            elif type(self._exclusion_buttons) == list:
                for n in self._exclusion_buttons:
                    if type(n) != float:
                        raise rospy.ROSException('Exclusion buttons must be an'
                                                 ' integer index to the joystick button')
        else:
            self._exclusion_buttons = list()

        self._joy_gain = 0.1
        if rospy.has_param('~joy_gain'):
            self._joy_gain = rospy.get_param('~joy_gain')

        # Retrieve the publish rate
        self._publish_rate = 100
        if rospy.has_param('~publish_rate'):
            self._publish_rate = rospy.get_param('~publish_rate')

        if self._publish_rate <= 0:
            raise rospy.ROSException('Invalid negative publish rate')

        self._pos_goal = self._gripper.closed_pos

        self._ratio_goal = 0.0

        if rospy.has_param('~kp'):
            self._kp = rospy.get_param('~kp')
        else:
            self._kp = 100.0

        if rospy.has_param('~ki'):
            self._ki = rospy.get_param('~ki')
        else:
            self._ki = 0.0

        if rospy.has_param('~kd'):
            self._kd = rospy.get_param('~kd')
        else:
            self._kd = 0.0

        self._controllers = dict()
        self._controllers[self._gripper.control_joint] = PIDRegulator(self._kp, self._ki, self._kd, 100)
        self._controllers[self._gripper.mimic_joint] = PIDRegulator(self._kp, self._ki, self._kd, 100)

        if self._gripper.control_joint is None:
            raise rospy.ROSException('Gripper cannot be controlled')

        self._limits = self._gripper.control_joint_limits

        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_callback)

        rate = rospy.Rate(self._publish_rate)

        while not rospy.is_shutdown():
            self._update()
            rate.sleep()

    def _update(self):
        # Compute the necessary effort for the requested joint position
        self._pos_goal = self._ratio_goal * (self._gripper.fully_open_pos - self._gripper.closed_pos) + \
                         self._gripper.closed_pos

        # Set the torque for the control joint
        error = self._pos_goal - self._gripper.control_joint_position
        torque = self._controllers[self._gripper.control_joint].regulate(error, rospy.get_time())
        self._gripper.set_controller_command(self._gripper.control_joint, torque)

        # Set the torque for the mimic joint
        error = self._pos_goal - self._gripper.mimic_joint_position
        torque = self._controllers[self._gripper.mimic_joint].regulate(error, rospy.get_time())
        self._gripper.set_controller_command(self._gripper.mimic_joint, torque)

        if self._gripper.is_parallel:
            pass

    def _joy_callback(self, joy):
        try:
            if self._deadman_button > -1:
                if not joy.buttons[self._deadman_button]:
                    return
            for n in self._exclusion_buttons:
                if joy.buttons[n] == 1:
                    return

            self._ratio_goal += self._joy_gain * (joy.buttons[self._open_button] - joy.buttons[self._close_button])
            if self._ratio_goal < 0:
                self._ratio_goal = 0.0
            if self._ratio_goal > 1:
                self._ratio_goal = 1.0
        except Exception, e:
            print 'Error occurred while parsing joystick input, check if the joy_id corresponds to the joystick ' \
                  'being used. message=%s' % str(e)


if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0] + '_gripper'
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    jt_controller = GripperController()

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
