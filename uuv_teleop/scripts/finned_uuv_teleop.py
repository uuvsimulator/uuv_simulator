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
from __future__ import print_function
import numpy
import rospy
import tf
import tf.transformations as trans

from sensor_msgs.msg import Joy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_thrusters.models import Thruster
from rospy.numpy_msg import numpy_msg


class FinnedUUVControllerNode:
    def __init__(self):
        print('FinnedUUVControllerNode: initializing node')

        self._ready = False

        # Test if any of the needed parameters are missing
        param_labels = ['n_fins', 'gain_roll', 'gain_pitch', 'gain_yaw',
                        'thruster_model', 'fin_topic_prefix',
                        'fin_topic_suffix', 'thruster_topic',
                        'axis_thruster', 'axis_roll', 'axis_pitch', 'axis_yaw']

        for label in param_labels:
            if not rospy.has_param('~%s' % label):
                raise rospy.ROSException('Parameter missing, label=%s' % label)

        # Number of fins
        self._n_fins = rospy.get_param('~n_fins')

        # Thruster joy axis gain
        self._thruster_joy_gain = 1
        if rospy.has_param('~thruster_joy_gain'):
            self._thruster_joy_gain = rospy.get_param('~thruster_joy_gain')

        # Read the vector for contribution of each fin on the change on
        # orientation
        gain_roll = rospy.get_param('~gain_roll')
        gain_pitch = rospy.get_param('~gain_pitch')
        gain_yaw = rospy.get_param('~gain_yaw')

        if len(gain_roll) != self._n_fins or len(gain_pitch) != self._n_fins \
            or len(gain_yaw) != self._n_fins:
            raise rospy.ROSException('Input gain vectors must have length '
                                     'equal to the number of fins')

        # Create the command angle to fin angle mapping
        self._rpy_to_fins = numpy.vstack((gain_roll, gain_pitch, gain_yaw)).T

        # Read the joystick mapping
        self._joy_axis = dict(axis_thruster=rospy.get_param('~axis_thruster'),
                              axis_roll=rospy.get_param('~axis_roll'),
                              axis_pitch=rospy.get_param('~axis_pitch'),
                              axis_yaw=rospy.get_param('~axis_yaw'))

        # Subscribe to the fin angle topics
        self._pub_cmd = list()
        self._fin_topic_prefix = rospy.get_param('~fin_topic_prefix')
        self._fin_topic_suffix = rospy.get_param('~fin_topic_suffix')
        for i in range(self._n_fins):
            topic = self._fin_topic_prefix + str(i) + self._fin_topic_suffix
            self._pub_cmd.append(
              rospy.Publisher(topic, FloatStamped, queue_size=10))

        # Create the thruster model object
        try:
            self._thruster_topic = rospy.get_param('~thruster_topic')
            self._thruster_params = rospy.get_param('~thruster_model')
            if 'max_thrust' not in self._thruster_params:
                raise rospy.ROSException('No limit to thruster output was given')
            self._thruster_model = Thruster.create_thruster(
                        self._thruster_params['name'], 0,
                        self._thruster_topic, None, None,
                        **self._thruster_params['params'])
        except:
            raise rospy.ROSException('Thruster model could not be initialized')

        # Subscribe to the joystick topic
        self.sub_joy = rospy.Subscriber('joy', numpy_msg(Joy),
                                        self.joy_callback)

        self._ready = True

    def joy_callback(self, msg):
        """Handle callbacks with joystick state."""

        if not self._ready:
            return

        try:
            thrust = max(0, msg.axes[self._joy_axis['axis_thruster']]) * \
                self._thruster_params['max_thrust'] * \
                self._thruster_joy_gain

            cmd_roll = msg.axes[self._joy_axis['axis_roll']]
            if abs(cmd_roll) < 0.2:
                cmd_roll = 0.0

            cmd_pitch = msg.axes[self._joy_axis['axis_pitch']]
            if abs(cmd_pitch) < 0.2:
                cmd_pitch = 0.0

            cmd_yaw = msg.axes[self._joy_axis['axis_yaw']]
            if abs(cmd_yaw) < 0.2:
                cmd_yaw = 0.0

            rpy = numpy.array([cmd_roll, cmd_pitch, cmd_yaw])
            fins = self._rpy_to_fins.dot(rpy)

            self._thruster_model.publish_command(thrust)

            for i in range(self._n_fins):
                cmd = FloatStamped()
                cmd.data = fins[i]
                self._pub_cmd[i].publish(cmd)

            if not self._ready:
                return
        except Exception as e:
            print('Error occurred while parsing joystick input, check '
                  'if the joy_id corresponds to the joystick ' 
                  'being used. message={}'.format(e))


if __name__ == '__main__':
    print('starting FinnedUUVControllerNode.py')
    rospy.init_node('finned_uuv_teleop')

    try:
        node = FinnedUUVControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
