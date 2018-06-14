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

import numpy as np
from uuv_manipulators_msgs.msg import EndPointState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import PyKDL


class EndEffectorState(object):
    def __init__(self):
        self._pose = {'position': [0, 0, 0],
                      'orientation': [0, 0, 0, 0]}

        self._velocity = {'linear': [0, 0, 0],
                          'angular': [0, 0, 0]}

        self._effort = {'force': [0, 0, 0],
                        'torque': [0, 0, 0]}

    @property
    def pose(self):
        return self._pose

    @property
    def position(self):
        return self._pose['position']

    @position.setter
    def position(self, value):
        assert value.size == 3
        self._pose['position'] = value

    @property
    def orientation(self):
        return self._pose['orientation']

    @orientation.setter
    def orientation(self, value):
        assert value.size == 4
        self._pose['orientation'] = value

    @property
    def linear_velocity(self):
        return self._velocity['linear']

    @linear_velocity.setter
    def linear_velocity(self, value):
        assert value.size == 3
        self._velocity['linear'] = value

    @property
    def angular_velocity(self):
        return self._velocity['angular']

    @angular_velocity.setter
    def angular_velocity(self, value):
        assert value.size == 3
        self._velocity['angular'] = value

    @property
    def force(self):
        return self._effort['force']

    @force.setter
    def force(self, value):
        assert value.size == 3
        self._effort['force'] = value

    @property
    def torque(self):
        return self._effort['torque']

    @torque.setter
    def torque(self, value):
        assert value.size == 3
        self._effort['torque'] = value

    def to_msg(self):
        pose = Pose()
        pose.position.x = self._pose['position'][0]
        pose.position.y = self._pose['position'][1]
        pose.position.z = self._pose['position'][2]

        pose.orientation.x = self._pose['orientation'][0]
        pose.orientation.y = self._pose['orientation'][1]
        pose.orientation.z = self._pose['orientation'][2]
        pose.orientation.w = self._pose['orientation'][3]

        return pose

    def to_frame(self):
        quaternion = (self._pose['orientation'][0],
                      self._pose['orientation'][1],
                      self._pose['orientation'][2],
                      self._pose['orientation'][3])

        euler = euler_from_quaternion(quaternion)
        frame = PyKDL.Frame(PyKDL.Rotation.RPY(euler[0], euler[1], euler[2]),
                            PyKDL.Vector(self._pose['position'][0],
                                         self._pose['position'][1],
                                         self._pose['position'][2]))
        return frame

    def to_kdl_twist(self):
        vel = PyKDL.Vector(self._velocity['linear'][0],
                           self._velocity['linear'][1],
                           self._velocity['linear'][2])
        rot = PyKDL.Vector(self._velocity['angular'][0],
                           self._velocity['angular'][1],
                           self._velocity['angular'][2])
        return PyKDL.Twist(vel, rot)

    def parse_message(self, msg):
        self._pose['position'] = [msg.pose.position.x,
                                  msg.pose.position.y,
                                  msg.pose.position.z]

        self._pose['orientation'] = [msg.pose.orientation.x,
                                     msg.pose.orientation.y,
                                     msg.pose.orientation.z,
                                     msg.pose.orientation.w]

        self._velocity['linear'] = [msg.twist.linear.x,
                                    msg.twist.linear.y,
                                    msg.twist.linear.z]

        self._velocity['angular'] = [msg.twist.angular.x,
                                     msg.twist.angular.y,
                                     msg.twist.angular.z]

        self._effort['force'] = [msg.wrench.force.x,
                                 msg.wrench.force.y,
                                 msg.wrench.force.z]

        self._effort['torque'] = [msg.wrench.torque.x,
                                  msg.wrench.torque.y,
                                  msg.wrench.torque.z]
