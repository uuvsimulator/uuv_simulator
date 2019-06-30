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
import numpy as np
import rospy
from uuv_control_msgs.srv import *
from uuv_control_interfaces.dp_controller_base import DPControllerBase


class ROVUnderActuatedPIDController(DPControllerBase):
    """
    This is an abstract class for PID-based controllers. The base class method
    update_controller must be overridden in other for a controller to work.
    """

    def __init__(self, *args):
        # Start the super class
        DPControllerBase.__init__(self, *args)
        self._logger.info('Initializing: Underactuated PID controller')
        # Proportional gains
        self._Kp = np.zeros(shape=(4, 4))
        # Derivative gains
        self._Kd = np.zeros(shape=(4, 4))
        # Integral gains
        self._Ki = np.zeros(shape=(4, 4))
        # Integrator component
        self._int = np.zeros(4)
        # Error for the vehicle pose
        self._error_pose = np.zeros(4)

        if rospy.has_param('~Kp'):
            Kp_diag = rospy.get_param('~Kp')
            if len(Kp_diag) == 4:
                self._Kp = np.diag(Kp_diag)
            else:
                raise rospy.ROSException('Kp matrix error: 4 coefficients '
                                         'needed')

        self._logger.info('Kp=' + str([self._Kp[i, i] for i in range(4)]))

        if rospy.has_param('~Kd'):
            Kd_diag = rospy.get_param('~Kd')
            if len(Kd_diag) == 4:
                self._Kd = np.diag(Kd_diag)
            else:
                raise rospy.ROSException('Kd matrix error: 4 coefficients '
                                         'needed')

        self._logger.info('Kd=' + str([self._Kd[i, i] for i in range(4)]))

        if rospy.has_param('~Ki'):
            Ki_diag = rospy.get_param('~Ki')
            if len(Ki_diag) == 4:
                self._Ki = np.diag(Ki_diag)
            else:
                raise rospy.ROSException('Ki matrix error: 4 coefficients '
                                         'needed')

        self._logger.info('Ki=' + str([self._Ki[i, i] for i in range(4)]))

        self._services['set_pid_params'] = rospy.Service(
            'set_pid_params',
            SetPIDParams,
            self.set_pid_params_callback)
        self._services['get_pid_params'] = rospy.Service(
            'get_pid_params',
            GetPIDParams,
            self.get_pid_params_callback)

        self._is_init = True
        self._logger.info('Underactuated PID controller ready!')

    def _reset_controller(self):
        super(DPPIDControllerBase, self)._reset_controller()
        self._error_pose = np.zeros(4)
        self._int = np.zeros(4)

    def set_pid_params_callback(self, request):
        kp = request.Kp
        kd = request.Kd
        ki = request.Ki
        if len(kp) != 4 or len(kd) != 4 or len(ki) != 4:
            return SetPIDParamsResponse(False)
        self._Kp = np.diag(kp)
        self._Ki = np.diag(ki)
        self._Kd = np.diag(kd)
        return SetPIDParamsResponse(True)

    def get_pid_params_callback(self, request):
        return GetPIDParamsResponse(
            [self._Kp[i, i] for i in range(4)],
            [self._Kd[i, i] for i in range(4)],
            [self._Ki[i, i] for i in range(4)])

    def update_controller(self):
        if not self.odom_is_init:
            return False
        if not self._is_init:
            return False
        # Update integrator
        cur_error_pose = np.array([self.error_pose_euler[0],
                                   self.error_pose_euler[1],
                                   self.error_pose_euler[2],
                                   self.error_pose_euler[5]])
        self._int += 0.5 * (cur_error_pose + self._error_pose) * self._dt
        # Store current pose error
        self._error_pose = cur_error_pose
        error_vel = np.array([self._errors['vel'][0],
                              self._errors['vel'][1],
                              self._errors['vel'][2],
                              self._errors['vel'][5]])
        ua_tau = np.dot(self._Kp, cur_error_pose) \
                        + np.dot(self._Kd, error_vel) \
                        + np.dot(self._Ki, self._int)
        self._tau = np.array([ua_tau[0], ua_tau[1], ua_tau[2], 0, 0, ua_tau[3]])
        self.publish_control_wrench(self._tau)
        return True

if __name__ == '__main__':
    print('Starting Underactuated PID Controller')
    rospy.init_node('rov_ua_pid_controller')

    try:
        node = ROVUnderActuatedPIDController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
