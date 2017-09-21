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
from uuv_control_interfaces import DPPIDControllerBase
from geometry_msgs.msg import Wrench, Vector3
from tf.transformations import quaternion_matrix


class ROV_NLPIDController(DPPIDControllerBase):
    """MIMO Nonlinear PID controller with acceleration feedback for the dynamic
    positioning of underwater vehicles.

    References
    ----------

    - Fossen, Thor I. Handbook of Marine Craft Hydrodynamics and Motion
    Control (April 8, 2011).
    """

    _LABEL = 'MIMO Nonlinear PID Controller with Acceleration Feedback'

    def __init__(self):
        DPPIDControllerBase.__init__(self, True)
        # Feedback acceleration gain
        self._Hm = np.eye(6)
        if rospy.has_param('Hm'):
            hm = rospy.get_param('Hm')
            if len(hm) == 6:
                self._Hm = self._vehicle_model.Mtotal +  np.diag(hm)
            else:
                raise rospy.ROSException('Invalid feedback acceleration gain coefficients')

        self._tau = np.zeros(6)
        # Acceleration feedback term
        self._accel_ff = np.zeros(6)
        # PID control vector
        self._pid_control = np.zeros(6)
        self._is_init = True

    def _reset_controller(self):
        super(ROV_NLPIDController, self)._reset_controller()
        self._accel_ff = np.zeros(6)
        self._pid_control = np.zeros(6)

    def update_controller(self):
        if not self._is_init:
            return False
        # Calculating the feedback acceleration vector for the control forces
        # from last iteration
        acc = self._vehicle_model.compute_acc(
            self._vehicle_model.to_SNAME(self._tau), use_sname=False)
        self._accel_ff = np.dot(self._Hm, acc)
        # Update PID control action
        self._pid_control = self.update_pid()
        # Publish control forces and torques
        self._tau = self._pid_control - self._accel_ff + self._vehicle_model.restoring_forces
        self.publish_control_wrench(self._tau)
        return True

if __name__ == '__main__':
    print('Starting NL PID Controller')
    rospy.init_node('rov_nl_pid_controller')

    try:
        node = ROV_NLPIDController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
