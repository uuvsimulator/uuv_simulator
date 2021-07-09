#!/usr/bin/env python3
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
from uuv_control_interfaces import DPPIDControllerBase
from uuv_control_msgs.srv import *

class ROV_MBFLController(DPPIDControllerBase):
    """
    Modelbased Feedback Linearization Controller
    Reference:
    Thor I. Fossen 2011
    Handbook of Marine Craft Hydrodynamics and Motion Control
    """
    _LABEL = 'Model-based Feedback Linearization Controller'

    def __init__(self):
        DPPIDControllerBase.__init__(self, True)
        self._logger.info('Initializing: ' + self._LABEL)

        # Control forces and torques
        self._tau = np.zeros(6)
        # PID control vector
        self._pid_control = np.zeros(6)
        self._is_init = True
        self._last_vel = np.zeros(6)
        self._last_t = None
        self._logger.info(self._LABEL + ' ready')

    def _reset_controller(self):
        super(ROV_MBFLController, self).reset_controller()
        self._pid_control = np.zeros(6)
        self._tau = np.zeros(6)

    def update_controller(self):
        if not self._is_init:
            return False

        t = rospy.get_time()
        if self._last_t is None:
            self._last_t = t
            self._last_vel = self._vehicle_model.to_SNAME(self._reference['vel']) 
            return False

        dt = t - self._last_t
        if dt <= 0:
            self._last_t = t
            self._last_vel = self._vehicle_model.to_SNAME(self._reference['vel']) 
            return False
        self._pid_control = self.update_pid()

        
        vel = self._vehicle_model.to_SNAME(self._reference['vel'])
        acc = (vel - self._last_vel) / dt

        self._vehicle_model._update_damping(vel)
        self._vehicle_model._update_coriolis(vel)
        self._vehicle_model._update_restoring(q=self._reference['rot'], use_sname=True)

        self._tau = np.dot(self._vehicle_model.Mtotal, acc) + \
                    np.dot(self._vehicle_model.Ctotal, vel) + \
                    np.dot(self._vehicle_model.Dtotal, vel) + \
                    self._vehicle_model.restoring_forces
                    
        # Publish control forces and torques
        self.publish_control_wrench(self._pid_control + self._vehicle_model.from_SNAME(self._tau))
        self._last_t = t
        self._last_vel = self._vehicle_model.to_SNAME(self._reference['vel'])
        return True


if __name__ == '__main__':
    print('Starting Modelbased Feedback Linearization Controller')
    rospy.init_node('rov_mb_fl_controller')

    try:
        node = ROV_MBFLController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
