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
from uuv_control_msgs.msg import Waypoint as WaypointMessage


class Waypoint(object):

    FINAL_WAYPOINT_COLOR = [1.0, 131.0 / 255, 0.0]
    OK_WAYPOINT = [31. / 255, 106. / 255, 226. / 255]
    FAILED_WAYPOINT = [1.0, 0.0, 0.0]

    def __init__(self, x=0, y=0, z=0, max_forward_speed=0, heading_offset=0,
        use_fixed_heading=False):
        self._x = x
        self._y = y
        self._z = z
        self._max_forward_speed = max_forward_speed
        self._heading_offset = heading_offset
        self._violates_constraint = False
        self._use_fixed_heading = use_fixed_heading

    def __eq__(self, other):
        return self._x == other.x and self._y == other.y and self._z == other.z

    def __ne__(self, other):
        return self._x != other.x or self._y != other.y or self._z != other.z

    def __str__(self):
        msg = '(x, y, z)= (%.2f, %.2f, %.2f) m\n' % (self._x, self._y, self._z)
        msg += 'Max. forward speed = %.2f\n' % self._max_forward_speed
        if self._use_fixed_heading:
            msg += 'Heading offset = %.2f degrees\n' % (self._heading_offset * 180 / np.pi)
        return msg

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    @property
    def pos(self):
        return np.array([self._x, self._y, self._z])

    @property
    def violates_constraint(self):
        return self._violates_constraint

    @violates_constraint.setter
    def violates_constraint(self, flag):
        self._violates_constraint = flag

    @property
    def max_forward_speed(self):
        return self._max_forward_speed

    @max_forward_speed.setter
    def max_forward_speed(self, vel):
        self._max_forward_speed = vel

    @property
    def heading_offset(self):
        return self._heading_offset

    @property
    def heading(self):
        return self._heading

    @heading.setter
    def heading(self, angle):
        self._heading = angle

    @property
    def using_heading_offset(self):
        return self._use_fixed_heading

    def get_color(self):
        return (self.FAILED_WAYPOINT if self._violates_constraint else self.OK_WAYPOINT)

    def get_final_color(self):
        return self.FINAL_WAYPOINT_COLOR

    def from_message(self, msg):
        self._x = msg.point.x
        self._y = msg.point.y
        self._z = msg.point.z
        self._max_forward_speed = msg.max_forward_speed
        self._use_fixed_heading = msg.use_fixed_heading
        self._heading_offset = msg.heading_offset

    def to_message(self):
        wp = WaypointMessage()
        wp.point.x = self._x
        wp.point.y = self._y
        wp.point.z = self._z
        wp.max_forward_speed = self._max_forward_speed
        wp.use_fixed_heading = self._use_fixed_heading
        wp.heading_offset = self._heading_offset
        return wp

    def dist(self, pos):
        return np.sqrt((self._x - pos[0])**2 +
                       (self._y - pos[1])**2 +
                       (self._z - pos[2])**2)
