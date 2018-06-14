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

import math
import numpy
import rospy
import tf.transformations as trans
from PID import PIDRegulator

from dynamic_reconfigure.server import Server
from uuv_control_cascaded_pid.cfg import PositionControlConfig
import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

class PositionControllerNode:
    def __init__(self):
        print('PositionControllerNode: initializing node')

        self.config = {}

        self.pos_des = numpy.zeros(3)
        self.quat_des = numpy.array([0, 0, 0, 1])

        self.initialized = False

        # Initialize pids with default parameters
        self.pid_rot = PIDRegulator(1, 0, 0, 1)
        self.pid_pos = PIDRegulator(1, 0, 0, 1)

        # ROS infrastructure
        self.sub_cmd_pose = rospy.Subscriber('cmd_pose', numpy_msg(geometry_msgs.PoseStamped), self.cmd_pose_callback)
        self.sub_odometry = rospy.Subscriber('odom', numpy_msg(Odometry), self.odometry_callback)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', geometry_msgs.Twist, queue_size=10)
        self.srv_reconfigure = Server(PositionControlConfig, self.config_callback)

    def cmd_pose_callback(self, msg):
        """Handle updated set pose callback."""
        # Just store the desired pose. The actual control runs on odometry callbacks
        p = msg.pose.position
        q = msg.pose.orientation
        self.pos_des = numpy.array([p.x, p.y, p.z])
        self.quat_des = numpy.array([q.x, q.y, q.z, q.w])

    def odometry_callback(self, msg):
        """Handle updated measured velocity callback."""
        if not bool(self.config):
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        p = numpy.array([p.x, p.y, p.z])
        q = numpy.array([q.x, q.y, q.z, q.w])

        if not self.initialized:
            # If this is the first callback: Store and hold latest pose.
            self.pos_des  = p
            self.quat_des = q
            self.initialized = True

        # Compute control output:
        t = msg.header.stamp.to_sec()

        # Position error
        e_pos_world = self.pos_des - p
        e_pos_body = trans.quaternion_matrix(q).transpose()[0:3,0:3].dot(e_pos_world)

        # Error quaternion wrt body frame
        e_rot_quat = trans.quaternion_multiply(trans.quaternion_conjugate(q), self.quat_des)

        if numpy.linalg.norm(e_pos_world[0:2]) > 5.0:
            # special case if we are far away from goal:
            # ignore desired heading, look towards goal position
            heading = math.atan2(e_pos_world[1],e_pos_world[0])
            quat_des = numpy.array([0, 0, math.sin(0.5*heading), math.cos(0.5*heading)])
            e_rot_quat = trans.quaternion_multiply(trans.quaternion_conjugate(q), quat_des)

        # Error angles
        e_rot = numpy.array(trans.euler_from_quaternion(e_rot_quat))

        v_linear = self.pid_pos.regulate(e_pos_body, t)
        v_angular = self.pid_rot.regulate(e_rot, t)

        # Convert and publish vel. command:
        cmd_vel = geometry_msgs.Twist()
        cmd_vel.linear = geometry_msgs.Vector3(*v_linear)
        cmd_vel.angular = geometry_msgs.Vector3(*v_angular)
        self.pub_cmd_vel.publish(cmd_vel)

    def config_callback(self, config, level):
        """Handle updated configuration values."""
        # Config has changed, reset PID controllers
        self.pid_pos = PIDRegulator(config['pos_p'], config['pos_i'], config['pos_d'], config['pos_sat'])
        self.pid_rot = PIDRegulator(config['rot_p'], config['rot_i'], config['rot_d'], config['rot_sat'])

        self.config = config

        return config


if __name__ == '__main__':
    print('starting PositionControl.py')
    rospy.init_node('position_control')

    try:
        node = PositionControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
