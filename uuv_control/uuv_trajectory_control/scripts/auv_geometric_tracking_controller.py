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

import geometry_msgs.msg as geometry_msgs
import uuv_control_msgs.msg as uuv_control_msgs
import tf
import tf.transformations as trans

from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_control_msgs.msg import TrajectoryPoint
from uuv_control_interfaces import DPControllerLocalPlanner


class AUVGeometricTrackingController:
    def __init__(self):
        print('AUVGeometricTrackingController initializing')

        self.local_planner = DPControllerLocalPlanner(full_dof=True, thrusters_only=False)

        # Reading the minimum thrust generated
        self.min_thrust = rospy.get_param('~min_thrust', 0)
        assert self.min_thrust > 0
        
        # Reading the maximum thrust generated
        self.max_thrust = rospy.get_param('~max_thrust', 0)
        assert self.max_thrust > 0 and self.max_thrust > self.min_thrust

        # Reading the thruster topic
        self.thruster_topic = rospy.get_param('~thruster_topic', 'thrusters/0/input')
        assert len(self.thruster_topic) > 0

        # Reading the thruster gain
        self.gain_thrust = rospy.get_param('~thrust_gain', 0.0)
        assert self.gain_thrust > 0

        # Reading the roll gain
        self.gain_roll = rospy.get_param('~gain_roll', 0.0)
        assert self.gain_roll > 0

        # Reading the pitch gain
        self.gain_pitch = rospy.get_param('~gain_pitch', 0.0)
        assert self.gain_pitch > 0

        # Reading the yaw gain
        self.gain_yaw = rospy.get_param('~gain_yaw', 0.0)
        assert self.gain_yaw > 0

        # Reading the saturation for the desired pitch
        self.desired_pitch_limit = rospy.get_param('~desired_pitch_limit', 15 * numpy.pi / 180)
        assert self.desired_pitch_limit > 0

        # Reading the saturation for yaw error
        self.yaw_error_limit = rospy.get_param('~yaw_error_limit', 1.0)
        assert self.yaw_error_limit > 0 

        # Reading the number of fins
        self.n_fins = rospy.get_param('~n_fins', 0)
        assert self.n_fins > 0

        # Reading the mapping for roll commands
        self.map_roll = rospy.get_param('~map_roll', [0, 0, 0, 0])
        assert isinstance(self.map_roll, list)
        assert len(self.map_roll) == self.n_fins
        
        # Reading the mapping for the pitch commands
        self.map_pitch = rospy.get_param('~map_pitch', [0, 0, 0, 0])
        assert isinstance(self.map_pitch, list)
        assert len(self.map_pitch) == self.n_fins

        # Reading the mapping for the yaw commands
        self.map_yaw = rospy.get_param('~map_yaw', [0, 0, 0, 0])
        assert isinstance(self.map_yaw, list)
        assert len(self.map_yaw) == self.n_fins

        self.max_fin_angle = rospy.get_param('~max_fin_angle', 0.0)
        assert self.max_fin_angle > 0

        # Reading the fin input topic prefix
        self.fin_topic_prefix = rospy.get_param('~fin_topic_prefix', 'fins')
        self.fin_topic_suffix = rospy.get_param('~fin_topic_suffix', 'input')

        self.rpy_to_fins = numpy.vstack((self.map_roll, self.map_pitch, self.map_yaw)).T

        self.pub_thrust = rospy.Publisher(
            self.thruster_topic, FloatStamped, queue_size=10)
        
        self.pub_cmd = list()

        for i in range(self.n_fins):
            topic = '%s/%d/%s' % (self.fin_topic_prefix, i, self.fin_topic_suffix)
            self.pub_cmd.append(
              rospy.Publisher(topic, FloatStamped, queue_size=10))

        self.odometry_sub = rospy.Subscriber(
            'odom', numpy_msg(Odometry), self.odometry_callback)

        self.reference_pub = rospy.Publisher(
            'reference', TrajectoryPoint, queue_size=1)

        # Publish error (for debugging)
        self.error_pub = rospy.Publisher(
            'error', TrajectoryPoint, queue_size=1)

    @staticmethod
    def unwrap_angle(t):
        return math.atan2(math.sin(t),math.cos(t))

    @staticmethod
    def vector_to_numpy(v):
        return numpy.array([v.x, v.y, v.z])

    @staticmethod
    def quaternion_to_numpy(q):
        return numpy.array([q.x, q.y, q.z, q.w])

    def odometry_callback(self, msg):
        """Handle odometry callback: The actual control loop."""

        # Update local planner's vehicle position and orientation
        pos = [msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.pose.pose.position.z]

        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]

        self.local_planner.update_vehicle_pose(pos, quat)

        # Compute the desired position
        t = rospy.Time.now().to_sec()
        des = self.local_planner.interpolate(t)

        # Publish the reference
        ref_msg = TrajectoryPoint()
        ref_msg.header.stamp = rospy.Time.now()
        ref_msg.header.frame_id = self.local_planner.inertial_frame_id
        ref_msg.pose.position = geometry_msgs.Vector3(*des.p)
        ref_msg.pose.orientation = geometry_msgs.Quaternion(*des.q)
        
        self.reference_pub.publish(ref_msg)

        p = self.vector_to_numpy(msg.pose.pose.position)

        q = self.quaternion_to_numpy(msg.pose.pose.orientation)
        R = trans.quaternion_matrix(q)[0:3, 0:3]
        v = self.vector_to_numpy(msg.twist.twist.linear)
        rpy = trans.euler_from_quaternion(q, axes='sxyz')

        # Compute tracking errors wrt world frame:
        e_p = des.p - p
        n_e_p = numpy.linalg.norm(e_p)

        # Generate error message
        error_msg = TrajectoryPoint()
        error_msg.header.stamp = rospy.Time.now()
        error_msg.header.frame_id = self.local_planner.inertial_frame_id
        error_msg.pose.position = geometry_msgs.Vector3(*e_p)
        error_msg.pose.orientation = geometry_msgs.Quaternion(
            *trans.quaternion_multiply(trans.quaternion_inverse(q), des.q))

        # Based on position tracking error: Compute desired orientation
        pitch_des = -math.atan2(e_p[2], numpy.linalg.norm(e_p[0:2]))
        # Limit desired pitch angle:
        pitch_des = max(-self.desired_pitch_limit, min(pitch_des, self.desired_pitch_limit))

        yaw_des = math.atan2(e_p[1], e_p[0])
        yaw_err = self.unwrap_angle(yaw_des - rpy[2])
        
        # Limit yaw effort
        yaw_err = min(self.yaw_error_limit, max(-self.yaw_error_limit, yaw_err))

        # Roll: P controller to keep roll == 0
        roll_control = self.gain_roll * rpy[0]

        # Pitch: P controller to reach desired pitch angle
        pitch_control = self.gain_pitch * self.unwrap_angle(pitch_des - rpy[1])

        # Yaw: P controller to reach desired yaw angle
        yaw_control = self.gain_yaw * yaw_err

        # Limit thrust
        thrust = min(self.max_thrust, self.gain_thrust * n_e_p)
        thrust = max(self.min_thrust, thrust)

        rpy = numpy.array([roll_control, pitch_control, yaw_control])

        # Transform orientation command into fin angle set points
        fins = self.rpy_to_fins.dot(rpy)

        # Check for saturation
        max_angle = max(numpy.abs(fins))
        if max_angle >= self.max_fin_angle:
            fins = fins*max_angle/self.max_fin_angle

        cmd = FloatStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.data = -1 * thrust
        self.pub_thrust.publish(cmd)

        for i in range(self.n_fins):
            cmd.data = min(fins[i], self.max_fin_angle)
            cmd.data = max(cmd.data, -self.max_fin_angle)
            self.pub_cmd[i].publish(cmd)

        self.error_pub.publish(error_msg)


if __name__ == '__main__':
    print('Starting AUV trajectory tracker')
    rospy.init_node('auv_geometric_tracking_controller')

    try:
        node = AUVGeometricTrackingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
