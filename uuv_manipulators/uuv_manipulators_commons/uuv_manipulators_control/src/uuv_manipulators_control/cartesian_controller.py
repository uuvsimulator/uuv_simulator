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
import numpy as np
import tf
import tf.transformations as trans
from tf_conversions import posemath
import PyKDL
from sensor_msgs.msg import Joy


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
        if not rospy.has_param('cartesian_controller/publish_rate'):
            self._publish_rate = rospy.get_param('cartesian_controller/publish_rate')

        if self._publish_rate <= 0:
            raise rospy.ROSException('Invalid negative publish rate')

        self._t_step = 0.01
        if rospy.has_param("~tstep"):
            self._t_step = rospy.get_param('~tstep')
            if self._t_step <= 0:
                raise rospy.ROSException('Invalid translational step')

        self._r_step = 0.01
        if rospy.has_param("~rstep"):
            self._r_step = rospy.get_param('~rstep')
            if self._r_step <= 0:
                raise rospy.ROSException('Invalid rotational step')

        # Default mapping for XBox 360 controller
        self._axes = dict(x=4, y=3, z=1, roll=0, pitch=7, yaw=6)
        if rospy.has_param('~axes'):
            axes = rospy.get_param('~axes')
            if type(axes) != dict:
                raise rospy.ROSException('Axes structure must be a dict')
            for tag in self._axes:
                if tag not in axes:
                    raise rospy.ROSException('Axes for %s missing' % tag)
                self._axes[tag] = axes[tag]

        # Default for the RB button of the XBox 360 controller
        self._deadman_button = 5
        if rospy.has_param('~deadman_button'):
            self._deadman_button = int(rospy.get_param('~deadman_button'))

        # If these buttons are pressed, the arm will not move
        if rospy.has_param('~exclusion_buttons'):
            self._exclusion_buttons = rospy.get_param('~exclusion_buttons')
            if type(self._exclusion_buttons) in [float, int]:
                self._exclusion_buttons = [int(self._exclusion_buttons)]
            elif type(self._exclusion_buttons) == list:
                for n in self._exclusion_buttons:
                    if type(n) not in [float, int]:
                        raise rospy.ROSException('Exclusion buttons must be an'
                                                 ' integer index to the joystick button')
        else:
            self._exclusion_buttons = list()

        # Default for the start button of the XBox 360 controller
        self._home_button = 7
        if rospy.has_param('~home_button'):
            self._home_button = int(rospy.get_param('~home_button'))

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

        rospy.set_param('cartesian_controller/name', self.LABEL)

        self._joint_effort_pub = dict()

        for joint in self._arm_interface.joint_names:
            self._joint_effort_pub[joint] = rospy.Publisher(
                self._arm_interface.namespace +
                joint + '/controller/command',
                Float64, queue_size=1)

        self._last_joy_update = rospy.get_time()
        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_callback)

    def _update(self, event):
        pass

    def _run(self):
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            self._update()
            rate.sleep()

    def _joy_callback(self, joy):
        self._command = np.zeros(6)
        if not joy.buttons[self._deadman_button] and self._deadman_button != -1:
            return
        for n in self._exclusion_buttons:
            if joy.buttons[n] == 1:
                return

        if joy.buttons[self._home_button] == 1:
            self._last_goal = self._arm_interface.get_config_in_ee_frame('home')
            self._last_joy_update = rospy.get_time()
            return


        self._command[0] = joy.axes[self._axes['x']] * self._t_step
        self._command[1] = joy.axes[self._axes['y']] * self._t_step
        self._command[2] = joy.axes[self._axes['z']] * self._t_step

        self._command[3] = joy.axes[self._axes['roll']] * self._r_step
        self._command[4] = joy.axes[self._axes['pitch']] * self._r_step
        self._command[5] = joy.axes[self._axes['yaw']] * self._r_step

        self._last_joy_update = rospy.get_time()

    def publish_joint_efforts(self, tau):
        # Publish torques
        t = np.asarray(tau).squeeze()
        for i, name in enumerate(self._arm_interface.joint_names):
            torque = Float64()
            torque.data = t[i]
            self._joint_effort_pub[name].publish(torque)
        # Update the time stamp
        self._last_controller_update = rospy.get_time()
