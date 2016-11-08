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
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import PyKDL
from uuv_manipulators_teleop import JoyNode
import numpy as np
from tf_conversions import posemath

# TODO Configure joystick mapping through the parameter server

class CartesianTeleopNode(JoyNode):
    def __init__(self):
        JoyNode.__init__(self)
        self._tStep = 0.001
        if rospy.has_param("~tstep"):
            self._tStep = rospy.get_param('~tstep')
            if self._tStep <= 0:
                raise rospy.ROSException('Invalid translational step')

        self._rStep = 0.001
        if rospy.has_param("~rstep"):
            self._rStep = rospy.get_param('~rstep')
            if self._rStep <= 0:
                raise rospy.ROSException('Invalid rotational step')

        if not rospy.has_param('cartesian_controller/teleop'):
            raise rospy.ROSException('Mapping for the cartesian teleop not available')

        self._teleop_mapping = rospy.get_param('cartesian_controller/teleop')

        self._cartesian_command_pub = rospy.Publisher(
            'cartesian_controller/command', PoseStamped, queue_size=10)
        self._ee_step = PoseStamped()

        self._update_rate = 100
        if rospy.has_param('cartesian_controller/update_rate'):
            self._update_rate = rospy.get_param('cartesian_controller/update_rate')

        rate = rospy.Rate(self._update_rate)
        while not rospy.is_shutdown():
            self._update()
            self._cartesian_command_pub.publish(self._ee_step)
            rate.sleep()

    def _update(self):
        if self._is_active():
            # TODO Find a way to put this mapping in the configuration file
            x = self._joy_input['axes']['right_y'] * self._tStep
            y = self._joy_input['axes']['right_x'] * self._tStep
            z = self._joy_input['axes']['left_y'] * self._tStep

            roll = self._joy_input['axes']['left_x'] * self._rStep
            pitch = self._joy_input['buttons']['cross_up'] * self._rStep - self._joy_input['buttons']['cross_down'] * self._rStep
            yaw = self._joy_input['buttons']['cross_left'] * self._rStep - self._joy_input['buttons']['cross_right'] * self._rStep
        else:
            x = 0
            y = 0
            z = 0
            roll = 0
            pitch = 0
            yaw = 0

        frame = PyKDL.Frame(PyKDL.Rotation.RPY(roll, pitch, yaw),
                            PyKDL.Vector(x, y, z))

        self._ee_step.header.stamp = rospy.Time.now()
        self._ee_step.pose = posemath.toMsg(frame)

if __name__ == "__main__":
    print 'Starting Cartesian Teleop'
    rospy.init_node('cartesian_teleop');
    try:
        node = CartesianTeleopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print 'CartesianTeleopNode::Exception'
    print 'Leaving CartesianTeleopNode'
