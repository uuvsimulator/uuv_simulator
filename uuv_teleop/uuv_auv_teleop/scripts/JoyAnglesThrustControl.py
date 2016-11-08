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

import numpy
import rospy
import tf
import tf.transformations as trans

from dynamic_reconfigure.server import Server
from uuv_auv_teleop.cfg import JoyAnglesThrustConfig
from sensor_msgs.msg import Joy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from rospy.numpy_msg import numpy_msg


class JoyAnglesThrustControllerNode:
    def __init__(self):
        print('JoyAnglesThrustControllerNode: initializing node')

        self.ready = False
        self.config = {}
        self.n_fins = 0
        self.rpy_to_fins = numpy.array([])

        self.pub_cmd = []

        # ROS infrastructure
        self.listener = tf.TransformListener()
        self.sub_joy = rospy.Subscriber(
          'joy', numpy_msg(Joy), self.joy_callback)
        self.srv_reconfigure = Server(
          JoyAnglesThrustConfig, self.config_callback)

    def joy_callback(self, msg):
        """Handle callbacks with joystick state."""

        if not self.ready:
            return

        thrust = msg.axes[self.config['axis_thruster']] * \
                 self.config['scale_thruster']
        rpy = numpy.array([msg.axes[self.config['axis_roll']],
                           msg.axes[self.config['axis_pitch']],
                           msg.axes[self.config['axis_yaw']]])

        fins = self.rpy_to_fins.dot(rpy)

        cmd = FloatStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.data = thrust
        self.pub_thrust.publish(cmd)

        for i in range(self.n_fins):
            cmd.data = fins[i]
            self.pub_cmd[i].publish(cmd)

        if not self.ready:
            return

    def config_callback(self, config, level):
        """Update configuration."""
        self.config = config
        print config

        self.n_fins = self.config['n_fins']

        print self.config['scale_pitch']
        roll = numpy.fromstring(
          self.config['scale_roll'], dtype=numpy.float, sep=',')
        pitch = numpy.fromstring(
          self.config['scale_pitch'], dtype=numpy.float, sep=',')
        yaw = numpy.fromstring(
          self.config['scale_yaw'], dtype=numpy.float, sep=',')

        self.rpy_to_fins = numpy.vstack((roll, pitch, yaw)).T
        print self.rpy_to_fins

        self.pub_thrust = rospy.Publisher(
          self.config['thruster_topic'], FloatStamped, queue_size=10)
        self.pub_cmd = []

        for i in range(self.n_fins):
            topic = self.config['fin_topic_prefix'] + str(i) + \
                    self.config['fin_topic_suffix']
            self.pub_cmd.append(
              rospy.Publisher(topic, FloatStamped, queue_size=10))

        self.ready = True
        return config


if __name__ == '__main__':
    print('starting JoyAnglesThrustControl.py')
    rospy.init_node('joy_angles_thrust_control')

    try:
        node = JoyAnglesThrustControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
