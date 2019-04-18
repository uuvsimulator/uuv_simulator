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

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Accel
from geometry_msgs.msg import Wrench
from rospy.numpy_msg import numpy_msg

class AccelerationControllerNode:
    def __init__(self):
        print('AccelerationControllerNode: initializing node')

        self.ready = False
        self.mass = 1.
        self.inertial_tensor = numpy.identity(3)
        self.mass_inertial_matrix = numpy.zeros((6, 6))

        # ROS infrastructure
        self.sub_accel = rospy.Subscriber(
          'cmd_accel', numpy_msg(Accel), self.accel_callback)
        self.sub_force = rospy.Subscriber(
          'cmd_force', numpy_msg(Accel), self.force_callback)
        self.pub_gen_force = rospy.Publisher(
          'thruster_manager/input', Wrench, queue_size=1)

        if not rospy.has_param("pid/mass"):
            raise rospy.ROSException("UUV's mass was not provided")

        if not rospy.has_param("pid/inertial"):
            raise rospy.ROSException("UUV's inertial was not provided")

        self.mass = rospy.get_param("pid/mass")
        self.inertial = rospy.get_param("pid/inertial")

        # update mass, moments of inertia
        self.inertial_tensor = numpy.array(
          [[self.inertial['ixx'], self.inertial['ixy'], self.inertial['ixz']],
           [self.inertial['ixy'], self.inertial['iyy'], self.inertial['iyz']],
           [self.inertial['ixz'], self.inertial['iyz'], self.inertial['izz']]])
        self.mass_inertial_matrix = numpy.vstack((
          numpy.hstack((self.mass*numpy.identity(3), numpy.zeros((3, 3)))),
          numpy.hstack((numpy.zeros((3, 3)), self.inertial_tensor))))

        print(self.mass_inertial_matrix)
        self.ready = True

    def force_callback(self, msg):
        if not self.ready:
            return

        # extract 6D force / torque vector from message
        force = numpy.array((
          msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z))
        torque = numpy.array((
          msg.accel.angular.x, msg.accel.angular.y, msg.accel.angular.z))
        force_torque = numpy.hstack((force, torque)).transpose()

        force_msg = Wrench()
        force_msg.force.x = force[0]
        force_msg.force.y = force[1]
        force_msg.force.z = force[2]

        force_msg.torque.x = torque[0]
        force_msg.torque.y = torque[1]
        force_msg.torque.z = torque[2]

        self.pub_gen_force.publish(force_msg)

    def accel_callback(self, msg):
        if not self.ready:
            return

        # extract 6D accelerations (linear, angular) from message
        linear = numpy.array((msg.linear.x, msg.linear.y, msg.linear.z))
        angular = numpy.array((msg.angular.x, msg.angular.y, msg.angular.z))
        accel = numpy.hstack((linear, angular)).transpose()

        # convert acceleration to force / torque
        force_torque = self.mass_inertial_matrix.dot(accel)

        force_msg = Wrench()
        force_msg.force.x = force_torque[0]
        force_msg.force.y = force_torque[1]
        force_msg.force.z = force_torque[2]

        force_msg.torque.x = force_torque[3]
        force_msg.torque.y = force_torque[4]
        force_msg.torque.z = force_torque[5]

        self.pub_gen_force.publish(force_msg)

if __name__ == '__main__':
    print('starting AccelerationControl.py')
    rospy.init_node('acceleration_control')

    try:
        node = AccelerationControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
