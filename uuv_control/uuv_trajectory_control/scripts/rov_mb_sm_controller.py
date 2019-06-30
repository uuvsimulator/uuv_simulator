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
from __future__ import print_function
import rospy
import numpy as np
from uuv_control_interfaces import DPControllerBase
from uuv_control_msgs.srv import *
from uuv_control_interfaces.vehicle import cross_product_operator

class ROV_MB_SMController(DPControllerBase):
    _LABEL = 'Model-based Sliding Mode Controller'

    def __init__(self):
        DPControllerBase.__init__(self, True)
        self._logger.info('Initializing: ' + self._LABEL)

        # Lambda - Slope of the Sliding Surface
        self._lambda = np.zeros(6)
        # Rho Constant - Vector of positive terms for assuring sliding surface reaching condition
        self._rho_constant = np.zeros(6)
        # k - PD gain (P term = k * lambda , D term = k)
        self._k = np.zeros(6)
        # c - slope of arctan (the greater, the more similar with the sign function)
        self._c = np.zeros(6)
        # Adapt slope - Adaptation gain for the estimation of uncertainties
        # and disturbances upper boundaries
        # adapt_slope = [proportional to surface distance, prop. to square
        # of pose errors, prop. to square of velocity errors]
        self._adapt_slope = np.zeros(3)
        # Rho_0 - rho_adapt treshold for drift prevention
        self._rho_0 = np.zeros(6)
        # Drift prevent - Drift prevention slope
        self._drift_prevent = 0

        if rospy.has_param('~lambda'):
            coefs = rospy.get_param('~lambda')
            if len(coefs) == 6:
                self._lambda = np.array(coefs)
            else:
                raise rospy.ROSException('lambda coefficients: 6 coefficients '
                                         'needed')
        print('lambda=', self._lambda)

        if rospy.has_param('~rho_constant'):
            coefs = rospy.get_param('~rho_constant')
            if len(coefs) == 6:
                self._rho_constant = np.array(coefs)
            else:
                raise rospy.ROSException('rho_constant coefficients: 6 coefficients '
                                         'needed')
        print('rho_constant=', self._rho_constant)

        if rospy.has_param('~k'):
            coefs = rospy.get_param('~k')
            if len(coefs) == 6:
                self._k = np.array(coefs)
            else:
                raise rospy.ROSException('k coefficients: 6 coefficients '
                                         'needed')
        print('k=', self._k)

        if rospy.has_param('~c'):
            coefs = rospy.get_param('~c')
            if len(coefs) == 6:
                self._c = np.array(coefs)
            else:
                raise rospy.ROSException('c coefficients: 6 coefficients '
                                         'needed')
        print('c=', self._c)

        if rospy.has_param('~adapt_slope'):
            coefs = rospy.get_param('~adapt_slope')
            if len(coefs) == 3:
                self._adapt_slope = np.array(coefs)
            else:
                raise rospy.ROSException('adapt_slope coefficients: 6 coefficients '
                                         'needed')
        print('adapt_slope=', self._adapt_slope)

        if rospy.has_param('~rho_0'):
            coefs = rospy.get_param('~rho_0')
            if len(coefs) == 6:
                self._rho_0 = np.array(coefs)
            else:
                raise rospy.ROSException('rho_0 coefficients: 6 coefficients '
                                         'needed')
        print('rho_0=', self._rho_0)

        if rospy.has_param('~drift_prevent'):
            scalar = rospy.get_param('~drift_prevent')
            if not isinstance(scalar, list):
                self._drift_prevent = scalar
            else:
                raise rospy.ROSException('drift_prevent needs to be a scalar value')

        print('drift_prevent=', self._drift_prevent)

        # Enable(1) / disable(0) integral term in the sliding surface
        if rospy.has_param('~enable_integral_term'):
            self._sliding_int = rospy.get_param('~enable_integral_term')
        else:
            self._sliding_int = 0

        # Enable(1) / disable(0) adaptive uncertainty upper boundaries for
        # robust control
        if rospy.has_param('~adaptive_bounds'):
            self._adaptive_bounds = rospy.get_param('~adaptive_bounds')
        else:
            self._adaptive_bounds = 1

        # Enable(1) / disable(0) constant uncertainty upper boundaries for
        # robust control
        if rospy.has_param('~constant_bound'):
            self._constant_bound = rospy.get_param('~constant_bound')
        else:
            self._constant_bound = 1

        # Enable(1) / disable(0) equivalent control term
        if rospy.has_param('~ctrl_eq'):
            self._ctrl_eq = rospy.get_param('~ctrl_eq')
        else:
            self._ctrl_eq = 1

        # Enable(1) / disable(0) linear control term
        if rospy.has_param('~ctrl_lin'):
            self._ctrl_lin = rospy.get_param('~ctrl_lin')
        else:
            self._ctrl_lin = 1

        # Enable(1) / disable(0) robust control term
        if rospy.has_param('~ctrl_robust'):
            self._ctrl_robust = rospy.get_param('~ctrl_robust')
        else:
            self._ctrl_robust = 1
        # Integrator component
        self._int = np.zeros(6)
        # Error for the vehicle pose
        self._error_pose = np.zeros(6)
        # Sliding Surface
        self._s_b = np.zeros(6)
        # Time derivative of the rotation matrix
        self._rotBtoI_dot = np.zeros(shape=(3, 3), dtype=float)
        # Linear acceleration estimation
        self._accel_linear_estimate_b = np.zeros(3)
        # Angular acceleration estimation
        self._accel_angular_estimate_b = np.zeros(3)
        # Acceleration estimation
        self._accel_estimate_b = np.zeros(6)
        # adaptive term of uncertainties upper bound estimation
        self._rho_adapt = np.zeros(6)
        # Upper bound for model uncertainties and disturbances
        self._rho_total = np.zeros(6)
        # Equivalent control
        self._f_eq = np.zeros(6)
        # Linear term of controller
        self._f_lin = np.zeros(6)
        # Robust control
        self._f_robust = np.zeros(6)
        # Total control
        self._tau = np.zeros(6)

        self._services['set_mb_sm_controller_params'] = rospy.Service(
            'set_mb_sm_controller_params',
            SetMBSMControllerParams,
            self.set_mb_sm_controller_params_callback)

        self._services['get_mb_sm_controller_params'] = rospy.Service(
            'get_mb_sm_controller_params',
            GetMBSMControllerParams,
            self.get_mb_sm_controller_params_callback)
        self._is_init = True
        self._logger.info(self._LABEL + ' ready')

    def _reset_controller(self):
        super(ROV_MB_SMController, self)._reset_controller()
        self._sliding_int = 0
        self._adaptive_bounds = 0
        self._constant_bound = 0
        self._ctrl_eq = 0
        self._ctrl_lin = 0
        self._ctrl_robust = 0
        self._prev_t = 0
        self._int = np.zeros(6)
        self._error_pose = np.zeros(6)
        self._s_b = np.zeros(6)
        self._rotBtoI_dot = np.zeros(shape=(3, 3), dtype=float)
        self._accel_linear_estimate_b = np.zeros(3)
        self._accel_angular_estimate_b = np.zeros(3)
        self._accel_estimate_b = np.zeros(6)
        self._rho_adapt = np.zeros(6)
        self._rho_total = np.zeros(6)
        self._f_eq = np.zeros(6)
        self._f_lin = np.zeros(6)
        self._f_robust = np.zeros(6)
        self._tau = np.zeros(6)

    def set_mb_sm_controller_params_callback(self, request):
        return SetMBSMControllerParamsResponse(True)

    def get_mb_sm_controller_params_callback(self, request):
        return GetMBSMControllerParamsResponse(
            self._lambda.tolist(),
            self._rho_constant.tolist(),
            self._k.tolist(),
            self._c.tolist(),
            self._adapt_slope.tolist(),
            self._rho_0.tolist(),
            self._drift_prevent)

    def update_controller(self):
        if not self._is_init:
            return False
        t = rospy.Time.now().to_sec()

        dt = t - self._prev_t
        if self._prev_t < 0.0:
            dt = 0.0

        # Update integrator
        self._int += 0.5 * (self.error_pose_euler - self._error_pose) * self._dt
        # Store current pose error
        self._error_pose = self.error_pose_euler

        # Get trajectory errors (reference - actual)
        e_p_linear_b = self._errors['pos']
        e_v_linear_b = self._errors['vel'][0:3]

        e_p_angular_b = self.error_orientation_rpy
        e_v_angular_b = self._errors['vel'][3:6]

        e_p_b = np.hstack((e_p_linear_b, e_p_angular_b))
        e_v_b = np.hstack((e_v_linear_b, e_v_angular_b))

        # Compute sliding surface s wrt body frame
        self._s_b = -e_v_b - np.multiply(self._lambda, e_p_b) \
                    - self._sliding_int * np.multiply(np.square(self._lambda)/4, self._int)

        # Acceleration estimate
        self._rotBtoI_dot = np.dot(cross_product_operator(self._vehicle_model._vel[3:6]), self._vehicle_model.rotBtoI)
        self._accel_linear_estimate_b = np.dot(
            self._vehicle_model.rotItoB, (self._reference['acc'][0:3] - \
                                          np.dot(self._rotBtoI_dot, self._vehicle_model._vel[0:3]))) + \
                                          np.multiply(self._lambda[0:3], e_v_linear_b) + \
                                          self._sliding_int * np.multiply(np.square(self._lambda[0:3]) / 4, e_p_linear_b)
        self._accel_angular_estimate_b = np.dot(self._vehicle_model.rotItoB, (self._reference['acc'][3:6] -
                                                np.dot(self._rotBtoI_dot, self._vehicle_model._vel[3:6]))) + \
                                                np.multiply(self._lambda[3:6], e_v_angular_b) + \
                                                self._sliding_int * np.multiply(np.square(self._lambda[3:6]) / 4,
                                                                                e_p_angular_b)
        self._accel_estimate_b = np.hstack((self._accel_linear_estimate_b, self._accel_angular_estimate_b))

        # Equivalent control
        acc = self._vehicle_model.to_SNAME(self._accel_estimate_b)
        self._f_eq = self._vehicle_model.compute_force(acc, use_sname=False)

        # Linear control
        self._f_lin = - np.multiply(self._k, self._s_b)

        # Uncertainties / disturbances upper boundaries for robust control
        self._rho_total = self._adaptive_bounds * self._rho_adapt + self._constant_bound * self._rho_constant

        # Adaptation law
        self._rho_adapt = self._rho_adapt + \
                          (self._adapt_slope[0] * np.abs(self._s_b) +
                          (self._adapt_slope[1] * np.abs(self._s_b) * np.abs(e_p_b) * np.abs(e_p_b)) +
                          (self._adapt_slope[2] * np.abs(self._s_b) * np.abs(e_v_b) * np.abs(e_v_b)) +
                           self._drift_prevent * (self._rho_0 - self._rho_adapt)) * dt

        # Robust control
        self._f_robust = - np.multiply(self._rho_total, (2 / np.pi) * np.arctan(np.multiply(self._c, self._s_b)))

        # Compute required forces and torques wrt body frame
        self._tau = self._ctrl_eq * self._f_eq + self._ctrl_lin * self._f_lin + self._ctrl_robust * self._f_robust

        self.publish_control_wrench(self._tau)

        self._prev_t = t

if __name__ == '__main__':
    print('Starting Model-based Sliding Mode Controller')
    rospy.init_node('rov_mb_sm_controller')

    try:
        node = ROV_MB_SMController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
