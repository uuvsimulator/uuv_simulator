#!/usr/bin/env python
# Copyright (c) 2016-2019 The UUV Simulator Authors.
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
from uuv_control_interfaces import DPControllerBase


class ROV_SFController(DPControllerBase):
    """
    Singularity-free tracking controller

    Reference
    ---------

    [1] O.-E. Fjellstad and T. I. Fossen, Singularity-free tracking of unmanned
        underwater vehicles in 6 DOF, Proceedings of 1994 33rd IEEE Conference
        on Decision and Control
    [2] G. Antonelli, "Underwater Robots," Springer Tracts in Advanced Robotics,
        2003
    """

    _LABEL = 'Singularity-free tracking controller'

    def __init__(self):
        DPControllerBase.__init__(self, True)
        self._tau = np.zeros(6)
        self._logger.info('Initializing: ' + self._LABEL)

        self._Kd = np.zeros(shape=(6, 6))

        if rospy.has_param('~Kd'):
            coefs = rospy.get_param('~Kd')
            if len(coefs) == 6:
                self._Kd = np.diag(coefs)
            else:
                raise rospy.ROSException('Kd coefficients: 6 coefficients '
                                         'needed')

        self._logger.info('Kd=\n' + str(self._Kd))

        # Build delta matrix
        self._delta = np.zeros(shape=(6, 6))

        l = rospy.get_param('~lambda', [0.0])

        if len(l) == 1:
            self._delta[0:3, 0:3] = l[0] * np.eye(3)
        elif len(l) == 3:
            self._delta[0:3, 0:3] = np.diag(l)
        else:
            raise rospy.ROSException(
                'lambda: either a scalar or a 3 element vector must be provided')

        c = rospy.get_param('~c', [0.0])

        if len(c) == 1:
            self._delta[3:6, 3:6] = c[0] * np.eye(3)
        elif len(c) == 3:
            self._delta[3:6, 3:6] = np.diag(c)
        else:
            raise rospy.ROSException(
                'c: either a scalar or a 3 element vector must be provided')

        self._logger.info('delta=\n' + str(self._delta))

        self._prev_t = None
        self._prev_vel_r = np.zeros(6)

        self._is_init = True
        self._logger.info(self._LABEL + ' ready')

    def update_controller(self):
        if not self._is_init:
            return False

        t = rospy.Time.now().to_sec()

        # Compute the generalized pose error vector using the quaternion
        # orientation error
        error = np.hstack((self._errors['pos'], self.error_orientation_quat))

        # Compute the virtual velocity signal
        vel_r = self._reference['vel'] + np.dot(self._delta, error)

        if self._prev_t is None:
            self._prev_t = t
            self._prev_vel_r = vel_r
            return False

        dt = t - self._prev_t

        if dt <= 0:
            return False

        # Compute virtual velocity error
        vel = self._vehicle_model.to_SNAME(self._vehicle_model.vel)
        # s = vel - self._vehicle_model.to_SNAME(vel_r)
        s = self._errors['vel'] + np.dot(self._delta, error)

        # Compute numerical derivative for virtual velocity signal
        d_vel_r = (vel_r - self._prev_vel_r) / dt

        sname_vel_r = self._vehicle_model.to_SNAME(vel_r)
        sname_d_vel_r = self._vehicle_model.to_SNAME(d_vel_r)
        self._vehicle_model._update_damping(vel)
        self._vehicle_model._update_coriolis(vel)
        self._vehicle_model._update_restoring(use_sname=True)

        self._tau = np.dot(self._vehicle_model.Mtotal, sname_d_vel_r) + \
            np.dot(self._vehicle_model.Ctotal, sname_vel_r) + \
            np.dot(self._vehicle_model.Dtotal, sname_vel_r) + \
            self._vehicle_model.restoring_forces

        # Update PID control action
        self.publish_control_wrench(
            self._vehicle_model.from_SNAME(self._tau) + np.dot(self._Kd, s))

        self._prev_t = t
        self._prev_vel_r = vel_r
        return True


if __name__ == '__main__':
    rospy.init_node('rov_sf_controller')

    try:
        node = ROV_SFController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
