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

import argparse
import rospy
import sys
import os
import numpy as np

import PyKDL
from uuv_manipulators_control import CartesianController


class JTCartesianController(CartesianController):
    """
    Jacobian transpose controller
    """

    LABEL = 'Jacobian transpose cartesian controller'
    def __init__(self):
        """
        Class constructor
        """
        CartesianController.__init__(self)
        # Retrieve the controller parameters from the parameter server
        Kd_tag = '~Kd'
        Kp_tag = '~Kp'
        if not rospy.has_param(Kd_tag):
            rospy.ROSException('Kd gain vector not available for tag=%s' % Kd_tag)
        if not rospy.has_param(Kp_tag):
            rospy.ROSException('Kp gain vector not available for tag=%s' % Kp_tag)

        self._Kd = rospy.get_param(Kd_tag)
        self._Kp = rospy.get_param(Kp_tag)

        rospy.loginfo('Kp=%s', str(self._Kp))
        rospy.loginfo('Kd=%s', str(self._Kd))

        # Initialization flag, to wait the end-effector get to the home
        # position
        self._is_init = False

        self._run()

    def _update(self):
        # Leave if ROS is not running or command is not valid
        if rospy.is_shutdown() or self._last_goal is None:
            return
        # Calculate the goal pose
        goal = self._get_goal()

        # End-effector's pose
        ee_pose = self._arm_interface.get_ee_pose_as_frame()
        # End-effector's velocity
        ee_vel = self._arm_interface.get_ee_vel_as_kdl_twist()
        # Calculate pose error
        error = PyKDL.diff(goal, ee_pose)
        # End-effector wrench to achieve target
        wrench = np.matrix(np.zeros(6)).T
        for i in range(len(wrench)):
            wrench[i] = -(self._Kp[i] * error[i] + self._Kd[i] * ee_vel[i])

        # Compute jacobian transpose
        JT = self._arm_interface.jacobian_transpose()
        # Calculate the torques for the joints
        tau = JT * wrench
        # Store current pose target
        self._last_goal = goal

        self.publish_goal()
        self.publish_joint_efforts(tau)

if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    jt_controller = JTCartesianController()

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
