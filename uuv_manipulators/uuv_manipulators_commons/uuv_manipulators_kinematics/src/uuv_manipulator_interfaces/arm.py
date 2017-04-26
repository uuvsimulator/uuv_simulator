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

from copy import deepcopy
import rospy

import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF
from kdl.kdl_parser import kdl_tree_from_urdf_model
from uuv_manipulators_msgs.msg import EndPointState
from uuv_kinematics_utils import EndEffectorState
from uuv_manipulator_interfaces.kin_chain import KinChainInterface
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ArmInterface(KinChainInterface):
    """Interface class to the arm."""

    STATE = ['moving', 'disabled']
    def __init__(self, namespace=None, arm_name=None, compute_fk_for_all=False):
        """Class constructor"""
        if None in [namespace, arm_name]:
            ns = [item for item in rospy.get_namespace().split('/') if len(item) > 0]
            if len(ns) != 2:
                rospy.ROSException('The controller must be called inside the namespace  the manipulator namespace')

            namespace = ns[0]
            arm_name = ns[1]

        param_path = '/' + namespace + '/arms/' + arm_name

        # Callbacks called in topic handlers
        self._callbacks = {}

        # Published topics
        self._pubTopics = {}
        # Subscribed topics
        self._subTopics = {}

        # Initializing the end-effector state
        self._endeffector_state = EndEffectorState()

        # Retrieve default configurations
        self._default_configs = dict()
        if rospy.has_param(param_path + '/default_configs'):
            self._default_configs = rospy.get_param(param_path + '/default_configs')
        else:
            self._default_configs = None

        base_link = None
        tip_link = None

        self._man_index = 0.0

        assert rospy.has_param(param_path + '/base_link'), 'Base link name not available in the namespace ' + namespace
        assert rospy.has_param(param_path + '/tip_link'), 'Tip link name not available in the namespace ' + namespace

        #  Retrieve the names of the base and tip links
        base_link = rospy.get_param(param_path + '/base_link')
        tip_link = rospy.get_param(param_path + '/tip_link')

        # Initialize kinematics chain interface
        KinChainInterface.__init__(self,
                                   name=arm_name,
                                   base=base_link,
                                   ee_link=tip_link,
                                   namespace=namespace,
                                   arm_name=arm_name,
                                   compute_fk_for_all=compute_fk_for_all)

        self.set_joint_names(self._joint_names)

        self._subTopics['joint_states'] = rospy.Subscriber(
            self._namespace + 'joint_states',
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

    def __del__(self):
        """Class destructor."""
        for topic in self._subTopics:
            self._subTopics[topic].unregister()
        for topic in self._pubTopics:
            self._pubTopics[topic].unregister()

    @property
    def endeffector_pose(self):
        return dict(position=self._endeffector_state.position,
                    orientation=self._endeffector_state.orientation)

    @property
    def endeffector_twist(self):
        return dict(linear=self._endeffector_state.linear_velocity,
                    angular=self._endeffector_state.angular_velocity)

    @property
    def endeffector_wrench(self):
        return dict(force=self._endeffector_state.force,
                    torque=self._endeffector_state.torque)

    @property
    def home(self):
        """
        Returns the joint configuration of the home states
        """
        if 'home' not in self._default_configs:
            return None
        else:
            return self._default_configs['home']

    @property
    def default_configs(self):
        return self._default_configs

    @property
    def man_index(self):
        """
        Returns the manipulability index
        """
        return self._man_index

    def _on_joint_states(self, msg):
        # Store the joint states
        self.update_joint_states(msg)

        # Update manipulability index
        self.update_man_index()
        # Update end-effector state
        self.update_endeffector_state()

        if 'joint_states' in self._callbacks:
            if len(self._callbacks['joint_states']):
                for fcn in self._callbacks['joint_states']:
                    fcn()

    def update_man_index(self):
        # Retrieve current jacobian matrix
        J = self.jacobian()
        det = np.linalg.det(np.dot(J, J.transpose()))
        self._man_index = 0.0
        if det > 0:
            self._man_index = np.sqrt(det)

    def update_endeffector_state(self):
        # Read joint states from the kinematics interface
        q = self.joint_angles
        qd = self.joint_velocities
        eff = self.joint_efforts

        # Compute the pose of all frames, if requested
        if self._compute_fk_for_all:
            for idx in xrange(self.n_links):
                self._frames[idx] = self.forward_position_kinematics(q, segment_idx=idx)
        # End effector pose
        pose = self.forward_position_kinematics(q)
        vel = self.forward_velocity_kinematics(q, qd)
        wrench = np.dot(self.jacobian(q), np.array(eff.values()))
        # Store everything in the end point state message
        self._endeffector_state.position = pose[0:3]
        self._endeffector_state.orientation = pose[3::]

        self._endeffector_state.linear_velocity = np.array([vel.vel.x(),
                                                            vel.vel.y(),
                                                            vel.vel.z()])
        self._endeffector_state.angular_velocity = np.array([vel.rot.x(),
                                                             vel.rot.y(),
                                                             vel.rot.z()])
        self._endeffector_state.force = np.array([wrench[0, 0],
                                                  wrench[0, 1],
                                                  wrench[0, 2]])
        self._endeffector_state.torque = np.array([wrench[0, 3],
                                                   wrench[0, 4],
                                                   wrench[0, 5]])

    def set_joint_names(self, jnt_names):
        self._joint_names = jnt_names

        # Adapting the joint names to the namespace
        if self._default_configs is not None:
            new_configs = {}
            for config in self._default_configs:
                new_configs[config] = {}
                for joint in self._default_configs[config]:
                    for j in self._joint_names:
                        if joint in j:
                            new_configs[config][j] = self._default_configs[config][joint]
            self._default_configs = new_configs

    def get_ee_pose_as_msg(self):
        return self._endeffector_state.to_msg()

    def get_ee_pose_as_frame(self):
        return self._endeffector_state.to_frame()

    def get_ee_vel_as_kdl_twist(self):
        return self._endeffector_state.to_kdl_twist()

    def get_config_in_ee_frame(self, config='home'):
        assert config in self._default_configs, 'Invalid default configuration'

        j_pos = self._default_configs[config]
        pose = self.forward_position_kinematics(j_pos)

        quaternion = (pose[3], pose[4], pose[5], pose[6])

        euler = euler_from_quaternion(quaternion)
        frame = PyKDL.Frame(PyKDL.Rotation.RPY(euler[0], euler[1], euler[2]),
                            PyKDL.Vector(pose[0], pose[1], pose[2]))
        return frame

    def get_config(self, config='home'):
        assert config in self._default_configs, 'Invalid default configuration'
        return self._default_configs[config]

    def add_callback(self, topic_name, function_handle):
        if topic_name not in self._subTopics:
            print 'ArmInterface - Invalid topic name'
            return

        if topic_name not in self._callbacks:
            self._callbacks[topic_name] = []

        self._callbacks[topic_name].append(function_handle)
