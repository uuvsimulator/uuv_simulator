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
from default_joy_reference_model import JoyReferenceModel


class FilteredJoyReferenceModel(JoyReferenceModel):
    """Filtered joystick output based on the second-order model in [1]
    [1] Dukan, Fredrik. "ROV Motion Control Systems", 2014, NTNU, PhD Thesis
    """
    LABEL = 'Filtered joystick reference model'

    def __init__(self):
        """Class constructor."""
        psi = 1
        if rospy.has_param('~psi'):
            psi = rospy.get_param('~psi')
            if psi < 0:
                raise rospy.ROSException('Invalid critical damping')
        wn = 1
        if rospy.has_param('~natural_frequency'):
            wn = rospy.get_param('~natural_frequency')
            if wn < 0:
                raise rospy.ROSException('Invalid natural frequency')

        self._psi = psi * np.ones(6)
        self._omega = wn * np.ones(6)

        self._tolerance = 0.01
        if rospy.has_param('~tolerance'):
            self._tolerance = rospy.get_param('~tolerance')
            if self._tolerance < 0:
                raise rospy.ROSException('Invalid tolerance')

        self._vel_ll = np.zeros(6)
        self._vel_l = np.zeros(6)
        self._vel = np.zeros(6)

        JoyReferenceModel.__init__(self)

        self._dt = 1. / self._publish_rate

    def _update(self):
        for i in range(6):
            if self._vjs[i] == 0 and abs(self._vel[i]) < self._tolerance:
                self._vel[i] = 0
            elif self._vjs[i] == 0 and abs(self._vel[i]) > self._tolerance:
                self._vel[i] = self._vel_l[i] * np.exp(-1 * self._dt)
            else:
                fac = self._dt**2 / \
                      (1 + 2 * self._dt * self._psi[i] * self._omega[i])
                a = 2 / (self._dt**2) + \
                    2 * self._psi[i] * self._omega[i] / self._dt
                self._vel[i] = fac * (self._omega[i]**2 * self._vjs[i] +
                                      a * self._vel_l[i] -
                                      (1 / self._dt**2) * self._vel_ll[i])

        # Update past velocity values
        self._vel_ll = self._vel_l
        self._vel_l = self._vel

        # Copy the result to the array that will be published
        self._vel_output = 100 * self._vel

if __name__ == '__main__':
    rospy.loginfo('Starting [filtered_joy_reference] node, '
                  'ns=%s' % rospy.get_namespace())

    rospy.init_node('filtered_joy_reference')
    joy_ref = FilteredJoyReferenceModel()
    joy_ref.run()
    rospy.spin()
