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
from uuv_control_interfaces import DPControllerBase
from uuv_control_msgs.srv import *


class ROV_NMB_SMController(DPControllerBase):
    """
    Model-free sliding mode controller based on the work published in [1] and
    [2], or model-free high order sliding mode controller.

    [1] Garcia-Valdovinos, Luis Govinda, et al. "Modelling, design and robust
        control of a remotely operated underwater vehicle." International
        Journal of Advanced Robotic Systems 11.1 (2014): 1.
    [2] Salgado-Jimenez, Tomas, Luis G. Garcia-Valdovinos, and Guillermo
        Delgado-Ramirez. "Control of ROVs using a Model-free 2nd-Order Sliding
        Mode Approach." Sliding Mode Control (2011): 347-368.
    """

    _LABEL = 'Non-model-based Sliding Mode Controller'

    def __init__(self):
        DPControllerBase.__init__(self, is_model_based=False)
        self._logger.info('Initializing: Non-model-based sliding mode controller')
        self._first_pass = True
        self._t_init = 0.0
        self._s_linear_b_init = np.array([0, 0, 0])
        self._s_angular_b_init = np.array([0, 0, 0])

        # 'K' gains (help in the initial transient)
        self._K = np.zeros(6)
        # Derivative gains
        self._Kd = np.zeros(6)
        # Robustness gains
        self._Ki = np.zeros(6)
        # Overall proportional gains
        self._slope = np.zeros(6)

        if rospy.has_param('~K'):
            coefs = rospy.get_param('~K')
            if len(coefs) == 6:
                self._K = np.array(coefs)
            else:
                raise rospy.ROSException('K coefficients: 6 coefficients '
                                         'needed')

        self._logger.info('K=' + str(self._K))

        if rospy.has_param('~Kd'):
            coefs = rospy.get_param('~Kd')
            if len(coefs) == 6:
                self._Kd = np.array(coefs)
            else:
                raise rospy.ROSException('Kd coefficients: 6 coefficients '
                                         'needed')

        self._logger.info('Kd=' + str(self._Kd))

        if rospy.has_param('~Ki'):
            coefs = rospy.get_param('~Ki')
            if len(coefs) == 6:
                self._Ki = np.array(coefs)
            else:
                raise rospy.ROSException('Ki coeffcients: 6 coefficients '
                                         'needed')
        self._logger.info('Ki=' + str(self._Ki))

        if rospy.has_param('~slope'):
            coefs = rospy.get_param('~slope')
            if len(coefs) == 6:
                self._slope = np.array(coefs)
            else:
                raise rospy.ROSException('Slope coefficients: 6 coefficients '
                                         'needed')

        self._logger.info('slope=' + str(self._slope))

        self._sat_epsilon = 0.8
        if rospy.has_param('~sat_epsilon'):
            self._sat_epsilon = max(0.0, rospy.get_param('~sat_epsilon'))

        self._logger.info('Saturation limits=' + str(self._sat_epsilon))

        self._summ_sign_sn_linear_b = np.array([0, 0, 0])
        self._summ_sign_sn_angular_b = np.array([0, 0, 0])

        self._prev_sign_sn_linear_b = np.array([0, 0, 0])
        self._prev_sign_sn_angular_b = np.array([0, 0, 0])

        self._tau = np.zeros(6)

        self._services['set_sm_controller_params'] = rospy.Service(
            'set_sm_controller_params',
            SetSMControllerParams,
            self.set_sm_controller_params_callback)
        self._services['get_sm_controller_params'] = rospy.Service(
            'get_sm_controller_params',
            GetSMControllerParams,
            self.get_sm_controller_params_callback)

        self._is_init = True
        self._logger.info('Non-model based sliding mode controller ready!')

    def _reset_controller(self):
        super(ROV_NMB_SMController, self)._reset_controller()
        self._first_pass = True
        self._t_init = 0.0
        self._s_linear_b_init = np.array([0, 0, 0])
        self._s_angular_b_init = np.array([0, 0, 0])
        self._prev_t = rospy.get_time()
        self._summ_sign_sn_linear_b = np.array([0, 0, 0])
        self._summ_sign_sn_angular_b = np.array([0, 0, 0])
        self._prev_sign_sn_linear_b = np.array([0, 0, 0])
        self._prev_sign_sn_angular_b = np.array([0, 0, 0])
        self._tau = np.zeros(6)

    def set_sm_controller_params_callback(self, request):
        return SetSMControllerParamsResponse(True)

    def get_sm_controller_params_callback(self, request):
        return GetSMControllerParamsResponse(
            self._K.tolist(),
            self._Kd.tolist(),
            self._Ki.tolist(),
            self._slope.tolist())

    def update_controller(self):
        if not self._is_init:
            return False
        t = rospy.Time.now().to_sec()

        dt = t - self._prev_t
        if self._prev_t < 0.0:
            dt = 0.0

        # Get trajectory errors (reference - actual)
        e_p_linear_b = self._errors['pos']
        e_v_linear_b = self._errors['vel'][0:3]

        e_p_angular_b = self.error_orientation_rpy # check this
        e_v_angular_b = self._errors['vel'][3:6]

        # Compute sliding surface s wrt body frame
        s_linear_b = -e_v_linear_b - np.multiply(self._slope[0:3],
                                                e_p_linear_b)
        s_angular_b = -e_v_angular_b - np.multiply(self._slope[3:6],
                                                  e_p_angular_b)

        # Compute exponential decay for transient improvement
        if self._first_pass == True:
            self._t_init, self._s_linear_b_init, self._s_angular_b_init = t, s_linear_b, s_angular_b
            self._first_pass = False

        sd_linear_b = np.multiply(self._s_linear_b_init,
                                  np.exp(-self._K[0:3] * (t - self._t_init)))
        sd_angular_b = np.multiply(self._s_angular_b_init,
                                   np.exp(-self._K[3:6] * (t - self._t_init)))

        # Compute sliding surface sn wrt body frame
        sn_linear_b = s_linear_b - sd_linear_b
        sn_angular_b = s_angular_b - sd_angular_b

        # Compute summation sign(sn) wrt body frame
        if self._prev_t > 0.0 and dt > 0.0:
            self._summ_sign_sn_linear_b = self._summ_sign_sn_linear_b + 0.5 * (
            self.sat(sn_linear_b, self._sat_epsilon) + self._prev_sign_sn_linear_b) * dt
            self._summ_sign_sn_angular_b = self._summ_sign_sn_angular_b + 0.5 * (
            self.sat(sn_angular_b, self._sat_epsilon) + self._prev_sign_sn_angular_b) * dt

        # Compute extended error wrt body frame
        sr_linear_b = sn_linear_b + np.multiply(self._Ki[0:3],
                                                self._summ_sign_sn_linear_b)
        sr_angular_b = sn_angular_b + np.multiply(self._Ki[3:6],
                                                  self._summ_sign_sn_angular_b)

        # Compute required forces and torques wrt body frame
        force_b = -np.multiply(self._Kd[0:3], sr_linear_b)
        torque_b = -np.multiply(self._Kd[3:6], sr_angular_b)

        self._tau = np.hstack((force_b, torque_b))

        self.publish_control_wrench(self._tau)

        self._prev_sign_sn_linear_b = self.sat(sn_linear_b, self._sat_epsilon)
        self._prev_sign_sn_angular_b = self.sat(sn_angular_b, self._sat_epsilon)
        self._prev_t = t

    @staticmethod
    def sat(value, epsilon=0.5):
        assert epsilon >= 0, 'Saturation constant must be greate or equal to zero'
        if epsilon == 0:
            return np.sign(value)

        vec = value / epsilon
        output = np.zeros(vec.size)
        for i in range(output.size):
            if vec[i] > epsilon:
                output[i] = 1
            elif vec[i] < -epsilon:
                output[i] = -1
            else:
                output[i] = vec[i]
        return output

if __name__ == '__main__':
    print('Starting Non-model-based Sliding Mode Controller')
    rospy.init_node('rov_nmb_sm_controller')

    try:
        node = ROV_NMB_SMController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
