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

import sys
import argparse
import rospy, os
from manipulator_kinematics import ManipulatorKinematics
from uuv_manipulators_msgs.msg import EndPointState
from geometry_msgs.msg import Vector3
import numpy as np


class EndPointStateNode:
    def __init__(self, namespace):
        # Storing the manipulator's namespace
        self._namespace = namespace
        if self._namespace[0] != '/':
            self._namespace = '/' + self._namespace

        # Calling the interface to the kinematics KDL interface
        self._kinematics = ManipulatorKinematics(self._namespace)

        # Creating a publisher for the end point state
        self._pub = rospy.Publisher(self._namespace + '/endpoint_state', EndPointState, queue_size=1)
        rate = rospy.Rate(500)
        try:
            while not rospy.is_shutdown():
                self._update()
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass

    def _update(self):
        state_msg = EndPointState
        # Read joint states from the kinematics interface
        q = self._kinematics.joint_angles.values()
        qd = self._kinematics.joint_velocities.values()
        eff = self._kinematics.joint_efforts.values()
        # Storing the pose
        pose = self._kinematics.forward_position_kinematics(q)
        state_msg.pose.position.x = pose[0]
        state_msg.pose.position.y = pose[1]
        state_msg.pose.position.z = pose[2]

        state_msg.pose.orientation.x = pose[3]
        state_msg.pose.orientation.y = pose[4]
        state_msg.pose.orientation.z = pose[5]
        state_msg.pose.orientation.w = pose[6]
        # Storing velocity
        vel = self._kinematics.forward_velocity_kinematics(qd)
        state_msg.twist.linear = Vector3(*vel.vel)
        state_msg.twist.angular = Vector3(*vel.rot)
        # Calculating wrench
        wrench = np.dot(self._kinematics.jacobian(q), np.array(eff))
        self._pub.publish(state_msg)

if __name__ == '__main__':
    # Create argument parser
    parser = argparse.ArgumentParser(description='End point state publisher')
    parser.add_argument('--namespace', metavar='NS', type=str)

    # Filter out the garbage
    arguments = [a for a in sys.argv if ':=' not in a and '.py' not in a]
    args = parser.parse_args(arguments)

    if not args.namespace:
        raise rospy.ROSException('Namespace was not given')

    namespace = args.namespace

    node_name = 'endpoint_state_publisher'
    rospy.init_node(node_name)
    node = EndPointStateNode(namespace)
    rospy.spin()
    rospy.loginfo('Shutting down, node=' + node_name)
