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
from uuv_manipulator_interfaces import ArmInterface
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64, Bool
from copy import deepcopy
import numpy as np
import tf
import tf.transformations as trans
from tf_conversions import posemath
import PyKDL


class CartesianController(object):
    LABEL = 'None'
    def __init__(self):
        # Timeout (to filter out inactivity)
        self._timeout = 0.5

        # Initializing the arm interface for the manipulator in the current
        # namespace
        self._arm_interface = ArmInterface()

        # Last goal for the end-effector pose/position
        self._last_goal = self._arm_interface.get_config_in_ee_frame('home')

        # Retrieve the publish rate
        self._publish_rate = 25
        if rospy.has_param('~publish_rate'):
            self._publish_rate = rospy.get_param('~publish_rate')

        if self._publish_rate <= 0:
            raise rospy.ROSException('Invalid negative publish rate')

        self._tau = 0.1

        # Get the transformation between the robot's base link and the
        # vehicle's base link
        self.listener = tf.TransformListener()

        # Get latest transform available
        latest = rospy.Time(0)
        base = self._arm_interface.namespace + 'base_link'
        self.listener.waitForTransform(base, self._arm_interface.base_link,
                                       latest, latest + rospy.Duration(100))
        [pos, quat] = self.listener.lookupTransform(
            base, self._arm_interface.base_link, latest)

        rot = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
        # Store transformation from the arm's base link and base
        self._trans = PyKDL.Frame(rot, PyKDL.Vector(pos[0], pos[1], pos[2]))

        # Velocity reference
        self._command = None

        # Last controller update
        self._last_controller_update = rospy.get_time()

        # Last velocity reference update time stamp
        self._last_reference_update = rospy.get_time()

        rospy.set_param('~name', self.LABEL)

        self._joint_effort_pub = dict()

        for joint in self._arm_interface.joint_names:
            self._joint_effort_pub[joint] = rospy.Publisher(
                self._arm_interface.namespace +
                joint + '/controller/command',
                Float64, queue_size=1)

        # Input velocity command subscriber, remap this topic to set a custom
        # command topic
        self._vel_command_sub = rospy.Subscriber(
            'cmd_vel', Twist, self._vel_command_callback)

        # Topic that will receive the flag if the home button was pressed. If
        # the flag is true, the manipulator's goal is set to the stow
        # configuration
        self._home_pressed_sub = rospy.Subscriber(
            'home_pressed', Bool, self._home_button_pressed)

        # Topic publishes the current goal set as reference to the manipulator
        self._goal_pub = rospy.Publisher(
            'reference', PoseStamped, queue_size=1)

        # Topic to publish a visual marker for visualization of the current
        # reference in RViz
        self._goal_marker_pub = rospy.Publisher(
            'reference_marker', Marker, queue_size=1)

    def _update(self, event):
        raise NotImplementedError()

    def _run(self):
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            self._update()
            rate.sleep()

    def _filter_input(self, state, cmd, dt):
        alpha = np.exp(- 1 * dt / self._tau)
        return state * alpha + (1.0 - alpha) * cmd

    def _get_goal(self):
        if self._command is None or rospy.get_time() - self._last_reference_update > 0.1:
            return self._last_goal       
        
        next_goal = deepcopy(self._last_goal)
        next_goal.p += PyKDL.Vector(self._command[0], self._command[1], self._command[2]) 

        q_step = trans.quaternion_from_euler(self._command[3],
                                             self._command[4],
                                             self._command[5])
        q_last = trans.quaternion_from_euler(*self._last_goal.M.GetRPY())
        q_next = trans.quaternion_multiply(q_last, q_step)

        next_goal.M = PyKDL.Rotation.Quaternion(*q_next)

        g_pos = [next_goal.p.x(), next_goal.p.y(), next_goal.p.z()]
        g_quat = next_goal.M.GetQuaternion()
        if self._arm_interface.inverse_kinematics(g_pos, g_quat) is not None:
            return next_goal
        else:
            print 'Next goal could not be resolved by the inv. kinematics solver.'
            return self._last_goal

    def _home_button_pressed(self, msg):
        if msg.data:
            self._command = np.zeros(6)
            self._last_goal = self._arm_interface.get_config_in_ee_frame('home')

    def _vel_command_callback(self, msg):
        dt = rospy.get_time() - self._last_reference_update
        if dt > 0.1:
            self._command = np.zeros(6)
            self._last_reference_update = rospy.get_time()
            return

        if self._command is None:
            self._command = np.zeros(6)
        
        self._command[0] = self._filter_input(self._command[0], msg.linear.x, dt) * dt
        self._command[1] = self._filter_input(self._command[1], msg.linear.y, dt) * dt
        self._command[2] = self._filter_input(self._command[2], msg.linear.z, dt) * dt

        self._command[3] = self._filter_input(self._command[3], msg.angular.x, dt) * dt
        self._command[4] = self._filter_input(self._command[4], msg.angular.y, dt) * dt
        self._command[5] = self._filter_input(self._command[5], msg.angular.z, dt) * dt

        self._last_reference_update = rospy.get_time()

    def publish_goal(self):
        # Publish the reference topic
        msg = PoseStamped()
        msg.header.frame_id = self._arm_interface.base_link
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = self._last_goal.p.x()
        msg.pose.position.y = self._last_goal.p.y()
        msg.pose.position.z = self._last_goal.p.z()

        msg.pose.orientation = Quaternion(*self._last_goal.M.GetQuaternion())

        self._goal_pub.publish(msg)

        marker = Marker()
        marker.header.frame_id = self._arm_interface.base_link
        marker.header.stamp = rospy.Time.now()
        marker.ns = self._arm_interface.arm_name
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        marker.pose.position = Vector3(self._last_goal.p.x(),
                                       self._last_goal.p.y(),
                                       self._last_goal.p.z())
        marker.pose.orientation = Quaternion(*self._last_goal.M.GetQuaternion())
        marker.scale = Vector3(0.2, 0.2, 0.2)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self._goal_marker_pub.publish(marker)

    def publish_joint_efforts(self, tau):
        # Publish torques
        t = np.asarray(tau).squeeze()
        for i, name in enumerate(self._arm_interface.joint_names):
            torque = Float64()
            torque.data = t[i]
            self._joint_effort_pub[name].publish(torque)
        # Update the time stamp
        self._last_controller_update = rospy.get_time()
