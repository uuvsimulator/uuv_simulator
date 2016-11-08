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
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from uuv_manipulators_msgs.srv import SolveIK
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

        # Retrieve the publish rate
        self._publish_rate = 25
        if not rospy.has_param('cartesian_controller/publish_rate'):
            self._publish_rate = rospy.get_param('cartesian_controller/publish_rate')

        if self._publish_rate <= 0:
            raise rospy.ROSException('Invalid negative publish rate')

        # Check if orientation will be controlled
        self._position_only = False
        if rospy.has_param('~position_only'):
            self._position_only = bool(rospy.get_param('~position_only'))

        # Check if cylindrical coordinates are to be used
        self._is_cylindrical = False
        if rospy.has_param('~is_cylindrical'):
            self._is_cylindrical = bool(rospy.get_param('~is_cylindrical'))

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
        # Subscribe to the twist reference topic
        self._command_sub = rospy.Subscriber('cartesian_controller/command',
                                             TwistStamped,
                                             self._command_callback,
                                             queue_size=1)

        # Last controller update
        self._last_controller_update = rospy.get_time()

        # Last goal for the end-effector pose/position
        self._last_goal = None

        try:
            # Wait for the IK solver service
            rospy.wait_for_service('ik_solver', timeout=2)
        except rospy.ROSException, e:
            print 'Service not available'

        try:
            self._ik_solver = rospy.ServiceProxy('ik_solver', SolveIK)
        except rospy.ServiceException, e:
            self._ik_solver = None

        rospy.set_param('cartesian_controller/name', self.LABEL)

        self._joint_effort_pub = dict()

        for joint in self._arm_interface.joint_names:
            self._joint_effort_pub[joint] = rospy.Publisher(
                self._arm_interface.namespace +
                joint + '/effort_controller/command',
                Float64, queue_size=1)

        self._controller_update_timer = rospy.Timer(
            rospy.Duration(1.0 / self._publish_rate), self._update)

        rospy.on_shutdown(self._on_shutdown)

    def _on_shutdown(self):
        self._controller_update_timer.shutdown()

    def _update(self, event):
        pass

    def _command_callback(self, msg):
        self._command = np.zeros(6)

        rot = self._trans.M
        twist = PyKDL.Twist(PyKDL.Vector(msg.twist.linear.x,
                                         msg.twist.linear.y,
                                         msg.twist.linear.z),
                            PyKDL.Vector(msg.twist.angular.x,
                                         msg.twist.angular.y,
                                         msg.twist.angular.z))
        twist = self._trans.M * twist

        self._command[0] = twist.vel.x()
        self._command[1] = twist.vel.y()
        self._command[2] = twist.vel.z()

        if not self._position_only:
            self._command[3] = twist.rot.x()
            self._command[4] = twist.rot.y()
            self._command[5] = twist.rot.z()

    def publish_joint_efforts(self, tau):
        # Publish torques
        t = np.asarray(tau).squeeze()
        for i, name in enumerate(self._arm_interface.joint_names):
            torque = Float64()
            torque.data = t[i]
            self._joint_effort_pub[name].publish(torque)
        # Update the time stamp
        self._last_controller_update = rospy.get_time()
