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
from dynamic_reconfigure.server import Server
import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry
import tf.transformations as trans
from rospy.numpy_msg import numpy_msg

# Modules included in this package
from PID import PIDRegulator
from uuv_control_cascaded_pid.cfg import VelocityControlConfig


class VelocityControllerNode:
    def __init__(self):
        print('VelocityControllerNode: initializing node')

        self.config = {}

        self.v_linear_des = numpy.zeros(3)
        self.v_angular_des = numpy.zeros(3)

        # Initialize pids with default parameters
        self.pid_angular = PIDRegulator(1, 0, 0, 1)
        self.pid_linear = PIDRegulator(1, 0, 0, 1)

        # ROS infrastructure
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', numpy_msg(geometry_msgs.Twist), self.cmd_vel_callback)
        self.sub_odometry = rospy.Subscriber('odom', numpy_msg(Odometry), self.odometry_callback)
        self.pub_cmd_accel = rospy.Publisher('cmd_accel', geometry_msgs.Accel, queue_size=10)
        self.srv_reconfigure = Server(VelocityControlConfig, self.config_callback)

    def cmd_vel_callback(self, msg):
        """Handle updated set velocity callback."""
        # Just store the desired velocity. The actual control runs on odometry callbacks
        v_l = msg.linear
        v_a = msg.angular
        self.v_linear_des = numpy.array([v_l.x, v_l.y, v_l.z])
        self.v_angular_des = numpy.array([v_a.x, v_a.y, v_a.z])

    def odometry_callback(self, msg):
        """Handle updated measured velocity callback."""
        if not bool(self.config):
            return

        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        v_linear = numpy.array([linear.x, linear.y, linear.z])
        v_angular = numpy.array([angular.x, angular.y, angular.z])

        if self.config['odom_vel_in_world']:
            # This is a temp. workaround for gazebo's pos3d plugin not behaving properly:
            # Twist should be provided wrt child_frame, gazebo provides it wrt world frame
            # see http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
            xyzw_array = lambda o: numpy.array([o.x, o.y, o.z, o.w])
            q_wb = xyzw_array(msg.pose.pose.orientation)
            R_bw = trans.quaternion_matrix(q_wb)[0:3, 0:3].transpose()

            v_linear = R_bw.dot(v_linear)
            v_angular = R_bw.dot(v_angular)

        # Compute compute control output:
        t = msg.header.stamp.to_sec()
        e_v_linear = (self.v_linear_des - v_linear)
        e_v_angular = (self.v_angular_des - v_angular)

        a_linear = self.pid_linear.regulate(e_v_linear, t)
        a_angular = self.pid_angular.regulate(e_v_angular, t)

        # Convert and publish accel. command:
        cmd_accel = geometry_msgs.Accel()
        cmd_accel.linear = geometry_msgs.Vector3(*a_linear)
        cmd_accel.angular = geometry_msgs.Vector3(*a_angular)
        self.pub_cmd_accel.publish(cmd_accel)

    def config_callback(self, config, level):
        """Handle updated configuration values."""
        # config has changed, reset PID controllers
        self.pid_linear = PIDRegulator(config['linear_p'], config['linear_i'], config['linear_d'], config['linear_sat'])
        self.pid_angular = PIDRegulator(config['angular_p'], config['angular_i'], config['angular_d'], config['angular_sat'])

        self.config = config

        return config


if __name__ == '__main__':
    print('starting VelocityControl.py')
    rospy.init_node('velocity_control')

    try:
        node = VelocityControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
