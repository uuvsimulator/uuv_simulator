#!/usr/bin/env python
# Copyright (c) 2014 The UUV Simulator Authors.
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
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_thrusters.models import Thruster
from tf.transformations import euler_from_quaternion


class AUVGeometricTrackingController(DPControllerBase):
    """
    Simple P-controller for finned AUVs.
    """
    def __init__(self, *args):
        # Start the super class
        DPControllerBase.__init__(self, *args)
        self._logger.info('Initializing: AUV Geometric Tracking Controller')

        # Thrust gain
        self._thrust_gain = 0.0

        if rospy.has_param('~thrust_gain'):
            self._thrust_gain = rospy.get_param('~thrust_gain')

        if self._thrust_gain <= 0:
            raise rospy.ROSException('Thrust gain must be greater than zero')

        self._logger.info('Thrust gain=' + str(self._thrust_gain))

        # Maximum absolute fin angle displacement should be given in radians
        self._max_fin_angle = 0.0

        if rospy.has_param('~max_fin_angle'):
            self._max_fin_angle = rospy.get_param('~max_fin_angle')

        if self._max_fin_angle <= 0.0:
            raise rospy.ROSException('Max. fin displacement angle must be greater than zero')

        # Proportional gain for the roll angle
        self._roll_gain = 0.0
        if rospy.has_param('~roll_gain'):
            self._roll_gain = rospy.get_param('~roll_gain')

        if self._roll_gain <= 0:
            raise rospy.ROSException('Roll gain must be greater than zero')

        self._logger.info('Roll gain=' + str(self._roll_gain))

        # Proportional gain for the pitch angle
        self._pitch_gain = 0.0
        if rospy.has_param('~pitch_gain'):
            self._pitch_gain = rospy.get_param('~pitch_gain')

        if self._pitch_gain <= 0:
            raise rospy.ROSException('Pitch gain must be greater than zero')

        self._logger.info('Pitch gain=' + str(self._pitch_gain))

        # Proportional gain for the yaw angle
        self._yaw_gain = 0.0
        if rospy.has_param('~yaw_gain'):
            self._yaw_gain = rospy.get_param('~yaw_gain')

        if self._yaw_gain <= 0:
            raise rospy.ROSException('Yaw gain must be greater than zero')

        self._logger.info('Yaw gain=' + str(self._yaw_gain))

        # Number of fins
        self._n_fins = 4
        if rospy.has_param('~n_fins'):
            self._n_fins = rospy.get_param('~n_fins')

        if self._n_fins <= 0:
            raise rospy.ROSException('Number of fins must be greater than zero')

        self._logger.info('Number of fins=' + str(self._n_fins))

        # Contribution of each fin to the orientation angles (roll, pitch and
        # yaw)
        self._fin_contribution_roll = [0, 0, 0, 0]
        self._fin_contribution_pitch = [0, 0, 0, 0]
        self._fin_contribution_yaw = [0, 0, 0, 0]

        if rospy.has_param('~fin_contribution_roll'):
            self._fin_contribution_roll = np.array(
                rospy.get_param('~fin_contribution_roll'))

        if self._fin_contribution_roll.size != self._n_fins:
            raise rospy.ROSException(
                'Number of elements in the roll contribution vector must be'
                ' equal to the number of fins')

        if np.max(self._fin_contribution_roll) > 1 or \
            np.min(self._fin_contribution_roll) < -1:
            raise rospy.ROSException('Values in the roll contribution vector'
                ' must be -1 <= i <= 1')

        if rospy.has_param('~fin_contribution_pitch'):
            self._fin_contribution_pitch = np.array(
                rospy.get_param('~fin_contribution_pitch'))

        if self._fin_contribution_pitch.size != self._n_fins:
            raise rospy.ROSException(
                'Number of elements in the pitch contribution vector must be'
                ' equal to the number of fins')

        if np.max(self._fin_contribution_pitch) > 1 or \
            np.min(self._fin_contribution_pitch) < -1:
            raise rospy.ROSException('Values in the pitch contribution vector'
                ' must be -1 <= i <= 1')

        if rospy.has_param('~fin_contribution_yaw'):
            self._fin_contribution_yaw = np.array(
                rospy.get_param('~fin_contribution_yaw'))

        if self._fin_contribution_yaw.size != self._n_fins:
            raise rospy.ROSException(
                'Number of elements in the yaw contribution vector must be'
                ' equal to the number of fins')

        if np.max(self._fin_contribution_yaw) > 1 or \
            np.min(self._fin_contribution_yaw) < -1:
            raise rospy.ROSException('Values in the yaw contribution vector'
                ' must be -1 <= i <= 1')

        # Generating vehicle orientation to fins conversion matrix
        self._rpy_to_fins = np.vstack((self._fin_contribution_roll,
                                       self._fin_contribution_pitch,
                                       self._fin_contribution_yaw)).T

        # Prefix to the fin command topic relative to the robot model
        self._fin_topic_prefix = 'fins/'

        if rospy.has_param('~fin_topic_prefix'):
            self._fin_topic_prefix = rospy.get_param('~fin_topic_prefix')

        # Fin command topic suffix
        self._fin_topic_suffix = '/input'

        if rospy.has_param('~fin_topic_suffix'):
            self._fin_topic_suffix = rospy.get_param('~fin_topic_suffix')

        # Initialize fin command publishers
        self._fin_pubs = list()
        for i in range(self._n_fins):
            topic = self._fin_topic_prefix + str(i) + self._fin_topic_suffix
            self._fin_pubs.append(rospy.Publisher(topic, FloatStamped,
                                                  queue_size=10))
            self._logger.info('Publishing command to #%i fin=%s' % (i, topic))

        # Suffix for the thruster topic relative to the robot model
        self._thruster_topic = 'thrusters/0/input'

        if rospy.has_param('~thruster_topic'):
            self._thruster_topic = rospy.get_param('~thruster_topic')

        # Get thruster parameters
        if not rospy.has_param('~thruster_model'):
            raise rospy.ROSException('Thruster parameters were not provided')

        self._thruster_params = rospy.get_param('~thruster_model')

        if 'max_thrust' not in self._thruster_params:
            raise rospy.ROSException('No limit to thruster output was given')
        self._thruster_model = Thruster.create_thruster(
                    self._thruster_params['name'], 0,
                    self._thruster_topic, None, None,
                    **self._thruster_params['params'])

        self._logger.info('Publishing to thruster=%s' % self._thruster_topic)

        self._is_init = True
        self._logger.info('AUV Geometric Tracking Controller ready!')

    @staticmethod
    def unwrap_angle(t):
        return np.arctan2(np.sin(t), np.cos(t))

    def update_controller(self):
        if not self._is_init:
            self._logger.info('Controller not initialized yet')
            return False

        world_pos_error = self._vehicle_model.rotBtoI.dot(self._errors['pos'])
        world_rpy = euler_from_quaternion(self._vehicle_model.quat, axes='sxyz')

        thrust_control = self._thrust_gain * np.linalg.norm(world_pos_error[0:2])

        # Check for saturation
        if np.abs(thrust_control) > self._thruster_params['max_thrust']:
            thrust_control = np.sign(thrust_control) * self._thruster_params['max_thrust']
        # Not using the custom wrench publisher that is configured per default
        # for thruster-actuated vehicles

        # Based on position tracking error: compute desired orientation
        pitch_des = -np.arctan2(world_pos_error[2],
                                np.linalg.norm(world_pos_error[0:2]))
        pitch_err = self.unwrap_angle(pitch_des - world_rpy[1])

        yaw_des = np.arctan2(world_pos_error[1], world_pos_error[0])
        yaw_err = self.unwrap_angle(yaw_des - world_rpy[2])

        rpy_control = np.array((self._roll_gain * world_rpy[0],
                                self._pitch_gain * pitch_err,
                                self._yaw_gain * yaw_err))
        fins_command = self._rpy_to_fins.dot(rpy_control)

        # Check for saturation
        max_angle = np.max(np.abs(fins_command))
        if max_angle >= self._max_fin_angle:
            for i in range(fins_command.size):
                if np.abs(fins_command[i]) > self._max_fin_angle:
                    fins_command[i] = np.sign(fins_command[i]) * self._max_fin_angle
        # Publishing the commands to thruster and fins
        self._thruster_model.publish_command(thrust_control)

        cmd = FloatStamped()
        cmd.header.stamp = rospy.Time.now()

        for i in range(self._n_fins):
            cmd.data = fins_command[i]
            self._fin_pubs[i].publish(cmd)

        return True

if __name__ == '__main__':
    print('Starting AUV Geometric Tracking Controller')
    rospy.init_node('auv_geometric_tracking_controller')

    try:
        # Setting up the local planner to generate trajectories in 6 DoF
        node = AUVGeometricTrackingController(False, list(), True)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
