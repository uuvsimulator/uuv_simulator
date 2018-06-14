#!/usr/bin/python
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

import PyKDL
import rospy
import numpy as np
from kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from uuv_manipulator_interfaces import ArmInterface
from uuv_manipulators_msgs.msg import EndPointState
from uuv_manipulators_msgs.srv import SolveIK, SolveIKResponse
from geometry_msgs.msg import Pose, Twist, Wrench, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import sys
import os
import numpy as np


class KinematicsService(object):
    def __init__(self):
        ns = [item for item in rospy.get_namespace().split('/') if len(item) > 0]
        if len(ns) != 2:
            rospy.ROSException('The controller must be called in the manipulator namespace')

        self._namespace = ns[0]
        self._arm_name = ns[1]

        if self._namespace[0] != '/':
            self._namespace = '/' + self._namespace

        if self._namespace[-1] != '/':
            self._namespace += '/'
        # The arm interface loads the parameters from the URDF file and
        # parameter server and initializes the KDL tree
        self._arm_interface = ArmInterface()

        self._base_link = self._arm_interface.base_link
        self._tip_link = self._arm_interface.tip_link

        self._tip_frame = PyKDL.Frame()

        # KDL Solvers
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._arm_interface.chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_interface.chain,
                                                   self._arm_interface._fk_p_kdl,
                                                   self._ik_v_kdl,
                                                   100,
                                                   1e-6)
        self._dyn_kdl = PyKDL.ChainDynParam(self._arm_interface.chain,
                                            PyKDL.Vector.Zero())

        # Add a callback function to calculate the end effector's state by each
        # update of the joint state
        self._arm_interface.add_callback('joint_states',
                                         self.on_update_endeffector_state)
        # Publish the current manipulability index at each update of the joint
        # states
        # self._arm_interface.add_callback('joint_states',
        #                                  self.publish_man_index)

        # Publish the end effector state
        self._endeffector_state_pub = rospy.Publisher('endpoint_state',
                                                      EndPointState,
                                                      queue_size=1)

        # Publish the manipulability index
        self._man_index_pub = rospy.Publisher('man_index',
                                              Float64,
                                              queue_size=1)

        self._services = dict()
        # Provide the service to calculate the inverse kinematics using the KDL solver
        self._services['ik'] = rospy.Service('ik_solver', SolveIK, self.solve_ik)

    @property
    def joint_names(self):
        return self._arm_interface.joint_names

    @property
    def joint_angles(self):
        return self._arm_interface.joint_angles

    @property
    def joint_velocities(self):
        return self._arm_interface.joint_velocities

    @property
    def joint_efforts(self):
        return self._arm_interface.joint_efforts

    @property
    def home(self):
        return self._arm_interface.home

    def solve_ik(self, req):
        out = SolveIKResponse()
        out.isValid = False
        out.joints = JointState()

        pos = [req.pose.position.x, req.pose.position.y, req.pose.position.z]
        orientation = [req.pose.orientation.x, req.pose.orientation.y,
                       req.pose.orientation.z, req.pose.orientation.w]

        result_ik = self._arm_interface.inverse_kinematics(pos, orientation)

        if result_ik is not None:
            for i, name in zip(range(len(self.joint_names)), self.joint_names):
                out.joints.name.append(name)
                out.joints.position.append(result_ik[i])
            out.isValid = True
        return out
        
    def publish_man_index(self):
        # Retrieve current jacobian matrix
        w_msg = Float64()
        w_msg.data = self._arm_interface.man_index
        self._man_index_pub.publish(w_msg)

    def on_update_endeffector_state(self):
        state_msg = EndPointState()
        # Store everything in the end point state message
        state_msg.pose.position.x = self._arm_interface.endeffector_pose['position'][0]
        state_msg.pose.position.y = self._arm_interface.endeffector_pose['position'][1]
        state_msg.pose.position.z = self._arm_interface.endeffector_pose['position'][2]

        state_msg.pose.orientation.x = self._arm_interface.endeffector_pose['orientation'][0]
        state_msg.pose.orientation.y = self._arm_interface.endeffector_pose['orientation'][1]
        state_msg.pose.orientation.z = self._arm_interface.endeffector_pose['orientation'][2]
        state_msg.pose.orientation.w = self._arm_interface.endeffector_pose['orientation'][3]

        state_msg.twist.linear.x = self._arm_interface.endeffector_twist['linear'][0]
        state_msg.twist.linear.y = self._arm_interface.endeffector_twist['linear'][1]
        state_msg.twist.linear.z = self._arm_interface.endeffector_twist['linear'][2]

        state_msg.twist.angular.x = self._arm_interface.endeffector_twist['angular'][0]
        state_msg.twist.angular.y = self._arm_interface.endeffector_twist['angular'][1]
        state_msg.twist.angular.z = self._arm_interface.endeffector_twist['angular'][2]

        state_msg.wrench.force.x = self._arm_interface.endeffector_wrench['force'][0]
        state_msg.wrench.force.y = self._arm_interface.endeffector_wrench['force'][1]
        state_msg.wrench.force.z = self._arm_interface.endeffector_wrench['force'][2]

        state_msg.wrench.torque.x = self._arm_interface.endeffector_wrench['torque'][0]
        state_msg.wrench.torque.y = self._arm_interface.endeffector_wrench['torque'][0]
        state_msg.wrench.torque.z = self._arm_interface.endeffector_wrench['torque'][0]

        self._endeffector_state_pub.publish(state_msg)

if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    manipulator_kin = KinematicsService()

    rospy.spin()
    rospy.loginfo('Shuting down [%s] node' % node_name)
