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
from copy import deepcopy
import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF
from kdl.kdl_parser import kdl_tree_from_urdf_model


class KinChainInterface(object):
    def __init__(self, name, base, ee_link, namespace=None, arm_name=None, compute_fk_for_all=False):
        # Joint states
        self._joint_angles = dict()
        self._joint_velocity = dict()
        self._joint_effort = dict()

        # Finger chain name
        self._name = name

        # Namespace
        if None in [namespace, arm_name]:
            ns = [item for item in rospy.get_namespace().split('/') if len(item) > 0]
            if len(ns) != 2:
                rospy.ROSException('The controller must be called inside the namespace  the manipulator namespace')

            self._namespace = ns[0]
            self._arm_name = ns[1]
        else:
            self._namespace = namespace
            self._arm_name = arm_name

        if self._namespace[0] != '/':
            self._namespace = '/' + self._namespace

        if self._namespace[-1] != '/':
            self._namespace += '/'

        # True if it has to compute the forward kinematics for all frames
        self._compute_fk_for_all = compute_fk_for_all

        # List of frames for each link
        self._frames = dict()

        # Load robot description (complete robot, including vehicle, if
        # available)
        self._robot_description = URDF.from_parameter_server(
            key=self._namespace + 'robot_description')

        # KDL tree of the whole structure
        self._kdl_tree = kdl_tree_from_urdf_model(self._robot_description)

        # Base link
        self._base_link = base

        # Tip link
        self._tip_link = ee_link

        # Read the complete link name, with robot's namespace, if existent
        for link in self._robot_description.links:
            if self._arm_name not in link.name:
                continue
            linkname = link.name.split('/')[-1]
            if self._base_link == linkname:
                self._base_link = link.name
            if self._tip_link == linkname:
                self._tip_link = link.name

        #  Get arm chain
        self._chain = self._kdl_tree.getChain(self._base_link,
                                              self._tip_link)

        # Partial tree from base to each link
        self._chain_part = dict()
        for link in self._robot_description.links:
            if link.name in self.link_names:
                self._chain_part[link.name] = self._kdl_tree.getChain(
                    self._base_link, link.name)

        # Get joint names from the chain
        # Joint names
        self._joint_names = list()
        for idx in xrange(self._chain.getNrOfSegments()):
            joint = self._chain.getSegment(idx).getJoint()
            # Not considering fixed joints
            if joint.getType() == 0:
                name = joint.getName()
                self._joint_names.append(name)
                self._joint_angles[name] = 0.0
                self._joint_velocity[name] = 0.0
                self._joint_effort[name] = 0.0

        # Jacobian solvers
        self._jac_kdl = PyKDL.ChainJntToJacSolver(self._chain)
        # Jacobian solver dictionary for partial trees
        self._jac_kdl_part = dict()
        for link in self._robot_description.links:
            if link.name in self.link_names:
                self._jac_kdl_part[link.name] = PyKDL.ChainJntToJacSolver(
                    self._chain_part[link.name])
        # Forward position kinematics solvers
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._chain)
        # Forward velocity kinematics solvers
        self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._chain)
        # Inverse kinematic solvers
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._chain,
                                                   self._fk_p_kdl,
                                                   self._ik_v_kdl,
                                                   100,
                                                   1e-6)

    @property
    def joint_names(self):
        """
        Returns the joint names
        """
        return self._joint_names

    @property
    def joint_angles(self):
        """
        Returns the joint angles
        """
        return deepcopy(self._joint_angles)

    @property
    def joint_velocities(self):
        """
        Returns the joint velocities
        """
        return deepcopy(self._joint_velocity)

    @property
    def joint_efforts(self):
        """
        Returns the joint efforts
        """
        return deepcopy(self._joint_effort)

    @property
    def joint_limits(self):
        limits = dict()
        for joint in self._robot_description.joints:
            if joint.name in self._joint_names:
                if joint.limit is not None:
                    limits[joint.name] = dict(effort=joint.limit.effort,
                                              lower=joint.limit.lower,
                                              upper=joint.limit.upper,
                                              velocity=joint.limit.velocity)
                else:
                    limits[joint.name] = None
        return limits

    @property
    def n_joints(self):
        """Number of joints."""
        return len(self._joint_names)

    @property
    def n_links(self):
        """Number of links."""
        return self._chain.getNrOfSegments()

    @property
    def link_names(self):
        """Link names."""
        names = list()
        for idx in xrange(self._chain.getNrOfSegments()):
            names.append(self._chain.getSegment(idx).getName())
        return names

    @property
    def full_namespace(self):
        """Full robot arm namespace name."""
        return self._namespace + self._arm_name

    @property
    def frames(self):
        """Current link frames."""
        return self._frames

    @property
    def namespace(self):
        """Underwater vehicle namespace."""
        return self._namespace

    @property
    def arm_name(self):
        """Name of the manipulator arm in tf."""
        return self._arm_name

    @property
    def chain(self):
        """KDL chain for the given base and tip links."""
        return self._chain

    @property
    def base_link(self):
        return self._base_link

    @property
    def tip_link(self):
        return self._tip_link

    @staticmethod
    def kdl_to_mat(data):
        """Convert a KDL matrix into Numpy matrix."""
        mat = np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i, j] = data[i, j]
        return mat

    def print_robot_description(self):
        nf_joints = 0
        for j in self._robot_description.joints:
            if j.type != 'fixed':
                nf_joints += 1
        print 'Base root=%s' % self._base_link
        print 'Tip link=%s' % self._tip_link
        print "URDF non-fixed joints: %d;" % nf_joints
        print "URDF total joints: %d" % len(self.n_joints)
        print "URDF links: %d" % len(self._robot_description.links)
        print "KDL joints: %d" % self._kdl_tree.getNrOfJoints()
        print "KDL segments: %d" % self._kdl_tree.getNrOfSegments()

    def print_chain(self):
        print 'Number of segments in chain=%d' % self._chain.getNrOfSegments()
        for idx in xrange(self._chain.getNrOfSegments()):
            print '* ' + self._chain.getSegment(idx).getName()

    def get_joint_angle(self, joint):
        assert joint in self._joint_angles, 'Invalid joint name'
        return self._joint_angles[joint]

    def get_joint_velocity(self, joint):
        assert joint in self._joint_velocity, 'Invalid joint name'
        return self._joint_effort[joint]

    def get_joint_effort(self, joint):
        assert joint in self._joint_velocity, 'Invalid joint name'
        return self._joint_effort[joint]

    def update_joint_states(self, states):
        for idx, name in enumerate(states.name):
            if len(states.velocity) == 0:
                return
            if name in self._joint_names:
                self._joint_angles[name] = states.position[idx]
                self._joint_velocity[name] = states.velocity[idx]
                self._joint_effort[name] = states.effort[idx]

    def joints_to_kdl(self, type, values=None, last_joint=None):
        if values is None:
            if type == 'positions':
                cur_type_values = self.joint_angles
            elif type == 'velocities':
                cur_type_values = self.joint_velocities
            elif type == 'torques':
                cur_type_values = self.joint_efforts
        else:
            cur_type_values = values

        if last_joint is None:
            last_idx = len(self.joint_names) - 1
            kdl_array = PyKDL.JntArray(self.n_joints)
        else:
            if last_joint not in self.joint_names:
                raise rospy.ROSException('Invalid joint name, joint_name=' + str(last_joint))
            last_idx = self.joint_names.index(last_joint)
            kdl_array = PyKDL.JntArray(last_idx + 1)
        for idx in range(last_idx + 1):
            name = self.joint_names[idx]
            if name in cur_type_values:
                kdl_array[idx] = cur_type_values[name]
        if type == 'velocities':
            kdl_array = PyKDL.JntArrayVel(kdl_array)
        return kdl_array

    def inertia(self):
        jnt_array = self.joints_to_kdl('positions')
        M_x_ee = PyKDL.JntSpaceInertiaMatrix(self.n_joints)
        self._dyn_kdl.JntToMass(jnt_array, M_x_ee)
        mat = self.kdl_to_mat(M_x_ee)
        return mat

    def jacobian(self, joint_values=None, end_link=None):
        """Compute the Jacobian for the current joint positions."""
        if end_link is not None:
            jnt_array = self.joints_to_kdl('positions', last_joint=self.joint_names[self.link_names.index(end_link)])
            jac = PyKDL.Jacobian(jnt_array.rows())
            jac_kdl_part = self._jac_kdl_part[end_link]
            jac_kdl_part.JntToJac(jnt_array, jac)
            mat = self.kdl_to_mat(jac)
            mat_add = np.matrix(np.zeros((mat.shape[0], mat.shape[0]-mat.shape[1])))
            mat = np.concatenate((mat,mat_add), axis=1)
        else:
            jnt_array = self.joints_to_kdl('positions')
            jac = PyKDL.Jacobian(jnt_array.rows())
            self._jac_kdl.JntToJac(
                jnt_array, jac)
            mat = self.kdl_to_mat(jac)
        return mat

    def jacobian_transpose(self, joint_values=None, end_link=None):
        """Return the Jacobian transpose."""
        if joint_values is None:
            joint_values = self.joint_angles
        if end_link is not None:
            return self.jacobian(end_link=end_link).T
        return self.jacobian(joint_values).T

    def jacobian_pseudo_inverse(self, joint_values=None, end_link=None):
        """Return the pseudo-inverse of the Jacobian matrix."""
        if joint_values is None:
            joint_values = self.joint_angles
        if end_link is not None:
            return np.linalg.pinv(self.jacobian(end_link=end_link))
        return np.linalg.pinv(self.jacobian(joint_values))

    def forward_position_kinematics(self, joint_values=None, segment_idx=-1):
        """Computation of the forward kinematics for this chain."""
        if joint_values is None:
            joint_values = self.joint_angles

        end_frame = PyKDL.Frame()
        self._fk_p_kdl.JntToCart(self.joints_to_kdl('positions', joint_values),
                                 end_frame, segment_idx)
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        return np.array([pos[0], pos[1], pos[2],
                         rot[0], rot[1], rot[2], rot[3]])

    def forward_velocity_kinematics(self, joint_values=None, joint_velocities=None):
        if joint_values is None:
            joint_values = self.joint_angles
        if joint_velocities is None:
            joint_values = self.joint_velocities

        q = PyKDL.JntArray(self.n_joints)
        q_dot = PyKDL.JntArray(self.n_joints)

        for idx in range(len(self.joint_names)):
            q[idx] = joint_values[self.joint_names[idx]]
            q_dot[idx] = joint_velocities[self.joint_names[idx]]

        vel_array = PyKDL.JntArrayVel(q, q_dot)

        end_frame = PyKDL.FrameVel()
        self._fk_v_kdl.JntToCart(vel_array, end_frame)

        return end_frame.GetTwist()

    def inverse_kinematics(self, position, orientation=None, seed=None):
        pos = PyKDL.Vector(position[0], position[1], position[2])
        if orientation is not None:
            rot = PyKDL.Rotation()
            rot = rot.Quaternion(orientation[0], orientation[1],
                                 orientation[2], orientation[3])
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self.n_joints)
        if seed is not None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        # Make IK Call
        if orientation is not None:
            goal_pose = PyKDL.Frame(rot, pos)
        else:
            goal_pose = PyKDL.Frame(pos)
        result_angles = PyKDL.JntArray(self.n_joints)

        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles))
            return result
        else:
            return None
