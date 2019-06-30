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
import math
import numpy as np
import rospy
from copy import deepcopy

import geometry_msgs.msg as geometry_msgs
import uuv_control_msgs.msg as uuv_control_msgs
import tf
import tf.transformations as trans

from nav_msgs.msg import Odometry
from uuv_thrusters.models import Thruster
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_control_msgs.msg import TrajectoryPoint
from uuv_control_interfaces import DPControllerLocalPlanner
import tf2_ros
from tf_quaternion.transformations import quaternion_matrix


class AUVGeometricTrackingController:
    def __init__(self):
        self.namespace = rospy.get_namespace().replace('/', '')
        rospy.loginfo('Initialize control for vehicle <%s>' % self.namespace)

        self.local_planner = DPControllerLocalPlanner(full_dof=True, thrusters_only=False,
            stamped_pose_only=False)

        self.base_link = rospy.get_param('~base_link', 'base_link')

        # Reading the minimum thrust generated
        self.min_thrust = rospy.get_param('~min_thrust', 0)
        assert self.min_thrust >= 0
        rospy.loginfo('Min. thrust [N]=%.2f', self.min_thrust)

        # Reading the maximum thrust generated
        self.max_thrust = rospy.get_param('~max_thrust', 0)
        assert self.max_thrust > 0 and self.max_thrust > self.min_thrust
        rospy.loginfo('Max. thrust [N]=%.2f', self.max_thrust)

        # Reading the thruster topic
        self.thruster_topic = rospy.get_param('~thruster_topic', 'thrusters/0/input')
        assert len(self.thruster_topic) > 0

        # Reading the thruster gain
        self.p_gain_thrust = rospy.get_param('~thrust_p_gain', 0.0)
        assert self.p_gain_thrust > 0

        self.d_gain_thrust = rospy.get_param('~thrust_d_gain', 0.0)
        assert self.d_gain_thrust >= 0

        # Reading the roll gain
        self.p_roll = rospy.get_param('~p_roll', 0.0)
        assert self.p_roll > 0

        # Reading the pitch P gain
        self.p_pitch = rospy.get_param('~p_pitch', 0.0)
        assert self.p_pitch > 0

        # Reading the pitch D gain
        self.d_pitch = rospy.get_param('~d_pitch', 0.0)
        assert self.d_pitch >= 0

        # Reading the yaw P gain
        self.p_yaw = rospy.get_param('~p_yaw', 0.0)
        assert self.p_yaw > 0

        # Reading the yaw D gain
        self.d_yaw = rospy.get_param('~d_yaw', 0.0)
        assert self.d_yaw >= 0

        # Reading the saturation for the desired pitch
        self.desired_pitch_limit = rospy.get_param('~desired_pitch_limit', 15 * np.pi / 180)
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

        # Retrieve the thruster configuration parameters
        self.thruster_config = rospy.get_param('~thruster_config')

        # Check if all necessary thruster model parameter are available
        thruster_params = ['conversion_fcn_params', 'conversion_fcn',
            'topic_prefix', 'topic_suffix', 'frame_base', 'max_thrust']
        for p in thruster_params:
            if p not in self.thruster_config:
                raise rospy.ROSException(
                    'Parameter <%s> for thruster conversion function is '
                    'missing' % p)

        # Setting up the thruster topic name
        self.thruster_topic = '/%s/%s/%d/%s' %  (self.namespace,
            self.thruster_config['topic_prefix'], 0,
            self.thruster_config['topic_suffix'])

        base = '%s/%s' % (self.namespace, self.base_link)

        frame = '%s/%s%d' % (self.namespace, self.thruster_config['frame_base'], 0)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo('Lookup: Thruster transform found %s -> %s' % (base, frame))
        trans = self.tf_buffer.lookup_transform(base, frame, rospy.Time(), rospy.Duration(5))
        pos = np.array([trans.transform.translation.x,
                        trans.transform.translation.y,
                        trans.transform.translation.z])
        quat = np.array([trans.transform.rotation.x,
                         trans.transform.rotation.y,
                         trans.transform.rotation.z,
                         trans.transform.rotation.w])
        rospy.loginfo('Thruster transform found %s -> %s' % (base, frame))
        rospy.loginfo('pos=' + str(pos))
        rospy.loginfo('rot=' + str(quat))

        # Read transformation from thruster
        self.thruster = Thruster.create_thruster(
            self.thruster_config['conversion_fcn'], 0,
            self.thruster_topic, pos, quat,
            **self.thruster_config['conversion_fcn_params'])

        rospy.loginfo('Thruster configuration=\n' + str(self.thruster_config))
        rospy.loginfo('Thruster input topic=' + self.thruster_topic)

        self.max_fin_angle = rospy.get_param('~max_fin_angle', 0.0)
        assert self.max_fin_angle > 0

        # Reading the fin input topic prefix
        self.fin_topic_prefix = rospy.get_param('~fin_topic_prefix', 'fins')
        self.fin_topic_suffix = rospy.get_param('~fin_topic_suffix', 'input')

        self.rpy_to_fins = np.vstack((self.map_roll, self.map_pitch, self.map_yaw)).T

        self.pub_cmd = list()

        for i in range(self.n_fins):
            topic = '%s/%d/%s' % (self.fin_topic_prefix, i, self.fin_topic_suffix)
            self.pub_cmd.append(
              rospy.Publisher(topic, FloatStamped, queue_size=10))

        self.odometry_sub = rospy.Subscriber(
            'odom', Odometry, self.odometry_callback)

        self.reference_pub = rospy.Publisher(
            'reference', TrajectoryPoint, queue_size=1)

        # Publish error (for debugging)
        self.error_pub = rospy.Publisher(
            'error', TrajectoryPoint, queue_size=1)

    @staticmethod
    def unwrap_angle(t):
        return math.atan2(math.sin(t),math.cos(t))

    @staticmethod
    def vector_to_np(v):
        return np.array([v.x, v.y, v.z])

    @staticmethod
    def quaternion_to_np(q):
        return np.array([q.x, q.y, q.z, q.w])

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
        ref_msg.velocity.linear = geometry_msgs.Vector3(*des.vel[0:3])
        ref_msg.velocity.angular = geometry_msgs.Vector3(*des.vel[3::])

        self.reference_pub.publish(ref_msg)

        p = self.vector_to_np(msg.pose.pose.position)
        forward_vel = self.vector_to_np(msg.twist.twist.linear)
        ref_vel = des.vel[0:3]

        q = self.quaternion_to_np(msg.pose.pose.orientation)
        rpy = trans.euler_from_quaternion(q, axes='sxyz')

        # Compute tracking errors wrt world frame:
        e_p = des.p - p
        abs_pos_error = np.linalg.norm(e_p)
        abs_vel_error = np.linalg.norm(ref_vel - forward_vel)

        # Generate error message
        error_msg = TrajectoryPoint()
        error_msg.header.stamp = rospy.Time.now()
        error_msg.header.frame_id = self.local_planner.inertial_frame_id
        error_msg.pose.position = geometry_msgs.Vector3(*e_p)
        error_msg.pose.orientation = geometry_msgs.Quaternion(
            *trans.quaternion_multiply(trans.quaternion_inverse(q), des.q))
        error_msg.velocity.linear = geometry_msgs.Vector3(*(des.vel[0:3] - self.vector_to_np(msg.twist.twist.linear)))
        error_msg.velocity.angular = geometry_msgs.Vector3(*(des.vel[3::] - self.vector_to_np(msg.twist.twist.angular)))

        # Based on position tracking error: Compute desired orientation
        pitch_des = -math.atan2(e_p[2], np.linalg.norm(e_p[0:2]))
        # Limit desired pitch angle:
        pitch_des = max(-self.desired_pitch_limit, min(pitch_des, self.desired_pitch_limit))

        yaw_des = math.atan2(e_p[1], e_p[0])
        yaw_err = self.unwrap_angle(yaw_des - rpy[2])

        # Limit yaw effort
        yaw_err = min(self.yaw_error_limit, max(-self.yaw_error_limit, yaw_err))

        # Roll: P controller to keep roll == 0
        roll_control = self.p_roll * rpy[0]

        # Pitch: P controller to reach desired pitch angle
        pitch_control = self.p_pitch * self.unwrap_angle(pitch_des - rpy[1]) + self.d_pitch * (des.vel[4] - msg.twist.twist.angular.y)

        # Yaw: P controller to reach desired yaw angle
        yaw_control = self.p_yaw * yaw_err + self.d_yaw * (des.vel[5] - msg.twist.twist.angular.z)

        # Limit thrust
        thrust = min(self.max_thrust, self.p_gain_thrust * np.linalg.norm(abs_pos_error) + self.d_gain_thrust * abs_vel_error)
        thrust = max(self.min_thrust, thrust)

        rpy = np.array([roll_control, pitch_control, yaw_control])

        # In case the world_ned reference frame is used, the convert it back
        # to the ENU convention to generate the reference fin angles
        rtf = deepcopy(self.rpy_to_fins)
        if self.local_planner.inertial_frame_id == 'world_ned':
            rtf[:, 1] *= -1
            rtf[:, 2] *= -1
        # Transform orientation command into fin angle set points
        fins = rtf.dot(rpy)

        # Check for saturation
        max_angle = max(np.abs(fins))
        if max_angle >= self.max_fin_angle:
            fins = fins * self.max_fin_angle / max_angle

        thrust_force = self.thruster.tam_column * thrust
        self.thruster.publish_command(thrust_force[0])

        cmd = FloatStamped()
        for i in range(self.n_fins):
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = '%s/fin%d' % (self.namespace, i)
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
