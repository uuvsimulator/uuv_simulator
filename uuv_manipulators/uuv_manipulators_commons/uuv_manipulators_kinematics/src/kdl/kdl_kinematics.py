#!/usr/bin/env python
#
# Provides wrappers for PyKDL kinematics.
#
# Copyright (c) 2012, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Kelsey Hawkins

import numpy as np

import rospy

import PyKDL as kdl

import hrl_geom.transformations as trans
from hrl_geom.pose_converter import PoseConv
from kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF

def create_kdl_kin(base_link, end_link, urdf_filename=None):
    if urdf_filename is None:
        robot = URDF.load_from_parameter_server(verbose=False)
    else:
        robot = URDF.load_xml_file(urdf_filename, verbose=False)
    return KDLKinematics(robot, base_link, end_link)

##
# Provides wrappers for performing KDL functions on a designated kinematic
# chain given a URDF representation of a robot.

class KDLKinematics(object):
    ##
    # Constructor
    # @param urdf URDF object of robot.
    # @param base_link Name of the root link of the kinematic chain.
    # @param end_link Name of the end link of the kinematic chain.
    # @param kdl_tree Optional KDL.Tree object to use. If None, one will be generated
    #                          from the URDF.
    def __init__(self, urdf, base_link, end_link, kdl_tree=None):
        if kdl_tree is None:
            kdl_tree = kdl_tree_from_urdf_model(urdf)
        self.tree = kdl_tree
        self.urdf = urdf

        base_link = base_link.split("/")[-1] # for dealing with tf convention
        end_link = end_link.split("/")[-1] # for dealing with tf convention
        self.chain = kdl_tree.getChain(base_link, end_link)
        self.base_link = base_link
        self.end_link = end_link

        # record joint information in easy-to-use lists
        self.joint_limits_lower = []
        self.joint_limits_upper = []
        self.joint_safety_lower = []
        self.joint_safety_upper = []
        self.joint_types = []
        for jnt_name in self.get_joint_names():
            jnt = urdf.joints[jnt_name]
            if jnt.limits is not None:
                self.joint_limits_lower.append(jnt.limits.lower)
                self.joint_limits_upper.append(jnt.limits.upper)
            else:
                self.joint_limits_lower.append(None)
                self.joint_limits_upper.append(None)
            if jnt.safety is not None:
                self.joint_safety_lower.append(jnt.safety.lower)
                self.joint_safety_upper.append(jnt.safety.upper)
            elif jnt.limits is not None:
                self.joint_safety_lower.append(jnt.limits.lower)
                self.joint_safety_upper.append(jnt.limits.upper)
            else:
                self.joint_safety_lower.append(None)
                self.joint_safety_upper.append(None)
            self.joint_types.append(jnt.joint_type)
        def replace_none(x, v):
            if x is None:
                return v
            return x
        self.joint_limits_lower = np.array([replace_none(jl, -np.inf) 
                                            for jl in self.joint_limits_lower])
        self.joint_limits_upper = np.array([replace_none(jl, np.inf) 
                                            for jl in self.joint_limits_upper])
        self.joint_safety_lower = np.array([replace_none(jl, -np.inf) 
                                            for jl in self.joint_safety_lower])
        self.joint_safety_upper = np.array([replace_none(jl, np.inf) 
                                            for jl in self.joint_safety_upper])
        self.joint_types = np.array(self.joint_types)
        self.num_joints = len(self.get_joint_names())

        self._fk_kdl = kdl.ChainFkSolverPos_recursive(self.chain)
        self._ik_v_kdl = kdl.ChainIkSolverVel_pinv(self.chain)
        self._ik_p_kdl = kdl.ChainIkSolverPos_NR(self.chain, self._fk_kdl, self._ik_v_kdl)
        self._jac_kdl = kdl.ChainJntToJacSolver(self.chain)
        self._dyn_kdl = kdl.ChainDynParam(self.chain, kdl.Vector.Zero())

    ##
    # @return List of link names in the kinematic chain.
    def get_link_names(self, joints=False, fixed=True):
        return self.urdf.get_chain(self.base_link, self.end_link, joints, fixed)

    ##
    # @return List of joint names in the kinematic chain.
    def get_joint_names(self, links=False, fixed=False):
        return self.urdf.get_chain(self.base_link, self.end_link,
                                   links=links, fixed=fixed)

    def get_joint_limits(self):
        return self.joint_limits_lower, self.joint_limits_upper

    def FK(self, q, link_number=None):
        if link_number is not None:
            end_link = self.get_link_names(fixed=False)[link_number]
        else:
            end_link = None
        homo_mat = self.forward(q, end_link)
        pos, rot = PoseConv.to_pos_rot(homo_mat)
        return pos, rot


    ##
    # Forward kinematics on the given joint angles, returning the location of the
    # end_link in the base_link's frame.
    # @param q List of joint angles for the full kinematic chain.
    # @param end_link Name of the link the pose should be obtained for.
    # @param base_link Name of the root link frame the end_link should be found in.
    # @return 4x4 numpy.mat homogeneous transformation
    def forward(self, q, end_link=None, base_link=None):
        link_names = self.get_link_names()
        if end_link is None:
            end_link = self.chain.getNrOfSegments()
        else:
            end_link = end_link.split("/")[-1]
            if end_link in link_names:
                end_link = link_names.index(end_link)
            else:
                print "Target segment %s not in KDL chain" % end_link
                return None
        if base_link is None:
            base_link = 0
        else:
            base_link = base_link.split("/")[-1]
            if base_link in link_names:
                base_link = link_names.index(base_link)
            else:
                print "Base segment %s not in KDL chain" % base_link
                return None
        base_trans = self._do_kdl_fk(q, base_link)
        if base_trans is None:
            print "FK KDL failure on base transformation."
        end_trans = self._do_kdl_fk(q, end_link)
        if end_trans is None:
            print "FK KDL failure on end transformation."
        return base_trans**-1 * end_trans

    def _do_kdl_fk(self, q, link_number):
        endeffec_frame = kdl.Frame()
        kinematics_status = self._fk_kdl.JntToCart(joint_list_to_kdl(q),
                                                   endeffec_frame,
                                                   link_number)
        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            return np.mat([[M[0,0], M[0,1], M[0,2], p.x()], 
                           [M[1,0], M[1,1], M[1,2], p.y()], 
                           [M[2,0], M[2,1], M[2,2], p.z()],
                           [     0,      0,      0,     1]])
        else:
            return None

    ##
    # Inverse kinematics for a given pose, returning the joint angles required
    # to obtain the target pose.
    # @param pose Pose-like object represeting the target pose of the end effector.
    # @param q_guess List of joint angles to seed the IK search.
    # @param min_joints List of joint angles to lower bound the angles on the IK search.
    #                   If None, the safety limits are used.
    # @param max_joints List of joint angles to upper bound the angles on the IK search.
    #                   If None, the safety limits are used.
    # @return np.array of joint angles needed to reach the pose or None if no solution was found.
    def inverse(self, pose, q_guess=None, min_joints=None, max_joints=None):
        pos, rot = PoseConv.to_pos_rot(pose)
        pos_kdl = kdl.Vector(pos[0,0], pos[1,0], pos[2,0])
        rot_kdl = kdl.Rotation(rot[0,0], rot[0,1], rot[0,2],
                               rot[1,0], rot[1,1], rot[1,2],
                               rot[2,0], rot[2,1], rot[2,2])
        frame_kdl = kdl.Frame(rot_kdl, pos_kdl)
        if min_joints is None:
            min_joints = self.joint_safety_lower
        if max_joints is None:
            max_joints = self.joint_safety_upper
        mins_kdl = joint_list_to_kdl(min_joints)
        maxs_kdl = joint_list_to_kdl(max_joints)
        ik_p_kdl = kdl.ChainIkSolverPos_NR_JL(self.chain, mins_kdl, maxs_kdl, 
                                              self._fk_kdl, self._ik_v_kdl)

        if q_guess == None:
            # use the midpoint of the joint limits as the guess
            lower_lim = np.where(np.isfinite(min_joints), min_joints, 0.)
            upper_lim = np.where(np.isfinite(max_joints), max_joints, 0.)
            q_guess = (lower_lim + upper_lim) / 2.0
            q_guess = np.where(np.isnan(q_guess), [0.]*len(q_guess), q_guess)

        q_kdl = kdl.JntArray(self.num_joints)
        q_guess_kdl = joint_list_to_kdl(q_guess)
        if ik_p_kdl.CartToJnt(q_guess_kdl, frame_kdl, q_kdl) >= 0:
            return np.array(joint_kdl_to_list(q_kdl))
        else:
            return None

    ##
    # Repeats IK for different sets of random initial angles until a solution is found
    # or the call times out.
    # @param pose Pose-like object represeting the target pose of the end effector.
    # @param timeout Time in seconds to look for a solution.
    # @return np.array of joint angles needed to reach the pose or None if no solution was found.
    def inverse_search(self, pose, timeout=1.):
        st_time = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - st_time < timeout:
            q_init = self.random_joint_angles()
            q_ik = self.inverse(pose, q_init)
            if q_ik is not None:
                return q_ik
        return None

    ##
    # Returns the Jacobian matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return 6xN np.mat Jacobian
    # @param pos Point in base frame where the jacobian is acting on.
    #            If None, we assume the end_link
    def jacobian(self, q, pos=None):
        j_kdl = kdl.Jacobian(self.num_joints)
        q_kdl = joint_list_to_kdl(q)
        self._jac_kdl.JntToJac(q_kdl, j_kdl)
        if pos is not None:
            ee_pos = self.forward(q)[:3,3]
            pos_kdl = kdl.Vector(pos[0]-ee_pos[0], pos[1]-ee_pos[1], 
                                  pos[2]-ee_pos[2])
            j_kdl.changeRefPoint(pos_kdl)
        return kdl_to_mat(j_kdl)

    ##
    # Returns the joint space mass matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return NxN np.mat Inertia matrix
    def inertia(self, q):
        h_kdl = kdl.JntSpaceInertiaMatrix(self.num_joints)
        self._dyn_kdl.JntToMass(joint_list_to_kdl(q), h_kdl)
        return kdl_to_mat(h_kdl)

    ##
    # Returns the cartesian space mass matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return 6x6 np.mat Cartesian inertia matrix
    def cart_inertia(self, q):
        H = self.inertia(q)
        J = self.jacobian(q)
        return np.linalg.inv(J * np.linalg.inv(H) * J.T)

    ##
    # Tests to see if the given joint angles are in the joint limits.
    # @param List of joint angles.
    # @return True if joint angles in joint limits.
    def joints_in_limits(self, q):
        lower_lim = self.joint_limits_lower
        upper_lim = self.joint_limits_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    ##
    # Tests to see if the given joint angles are in the joint safety limits.
    # @param List of joint angles.
    # @return True if joint angles in joint safety limits.
    def joints_in_safe_limits(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    ##
    # Clips joint angles to the safety limits.
    # @param List of joint angles.
    # @return np.array list of clipped joint angles.
    def clip_joints_safe(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.clip(q, lower_lim, upper_lim)

    ##
    # Returns a set of random joint angles distributed uniformly in the safety limits.
    # @return np.array list of random joint angles.
    def random_joint_angles(self):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        lower_lim = np.where(np.isfinite(lower_lim), lower_lim, -np.pi)
        upper_lim = np.where(np.isfinite(upper_lim), upper_lim, np.pi)
        zip_lims = zip(lower_lim, upper_lim)
        return np.array([np.random.uniform(min_lim, max_lim) for min_lim, max_lim in zip_lims])

    ##
    # Returns a difference between the two sets of joint angles while insuring
    # that the shortest angle is returned for the continuous joints.
    # @param q1 List of joint angles.
    # @param q2 List of joint angles.
    # @return np.array of wrapped joint angles for retval = q1 - q2
    def difference_joints(self, q1, q2):
        diff = np.array(q1) - np.array(q2)
        diff_mod = np.mod(diff, 2 * np.pi)
        diff_alt = diff_mod - 2 * np.pi 
        for i, continuous in enumerate(self.joint_types == 'continuous'):
            if continuous:
                if diff_mod[i] < -diff_alt[i]:
                    diff[i] = diff_mod[i]
                else:
                    diff[i] = diff_alt[i]
        return diff

    ##
    # Performs an IK search while trying to balance the demands of reaching the goal,
    # maintaining a posture, and prioritizing rotation or position.
    def inverse_biased(self, pose, q_init, q_bias, q_bias_weights, rot_weight=1., 
                       bias_vel=0.01, num_iter=100):
        # This code is potentially volitile
        q_out = np.mat(self.inverse_search(pose)).T
        for i in range(num_iter):
            pos_fk, rot_fk = PoseConv.to_pos_rot(self.forward(q_out))
            delta_twist = np.mat(np.zeros((6, 1)))
            pos_delta = pos - pos_fk
            delta_twist[:3,0] = pos_delta
            rot_delta = np.mat(np.eye(4))
            rot_delta[:3,:3] = rot * rot_fk.T
            rot_delta_angles = np.mat(trans.euler_from_matrix(rot_delta)).T
            delta_twist[3:6,0] = rot_delta_angles
            J = self.jacobian(q_out)
            J[3:6,:] *= np.sqrt(rot_weight)
            delta_twist[3:6,0] *= np.sqrt(rot_weight)
            J_tinv = np.linalg.inv(J.T * J + np.diag(q_bias_weights) * np.eye(len(q_init))) * J.T
            q_bias_diff = q_bias - q_out
            q_bias_diff_normed = q_bias_diff * bias_vel / np.linalg.norm(q_bias_diff)
            delta_q = q_bias_diff_normed + J_tinv * (delta_twist - J * q_bias_diff_normed)
            q_out += delta_q 
            q_out = np.mat(self.clip_joints_safe(q_out.T.A[0])).T
        return q_out

    ##
    # inverse_biased with random restarts.
    def inverse_biased_search(self, pos, rot, q_bias, q_bias_weights, rot_weight=1., 
                              bias_vel=0.01, num_iter=100, num_search=20):
        # This code is potentially volitile
        q_sol_min = []
        min_val = 1000000.
        for i in range(num_search):
            q_init = self.random_joint_angles()
            q_sol = self.inverse_biased(pos, rot, q_init, q_bias, q_bias_weights, rot_weight=1., 
                                        bias_vel=bias_vel, num_iter=num_iter)
            cur_val = np.linalg.norm(np.diag(q_bias_weights) * (q_sol - q_bias)) 
            if cur_val < min_val:
                min_val = cur_val
                q_sol_min = q_sol
        return q_sol_min
        

def kdl_to_mat(m):
    mat =  np.mat(np.zeros((m.rows(), m.columns())))
    for i in range(m.rows()):
        for j in range(m.columns()):
            mat[i,j] = m[i,j]
    return mat

def joint_kdl_to_list(q):
    if q == None:
        return None
    return [q[i] for i in range(q.rows())]

def joint_list_to_kdl(q):
    if q is None:
        return None
    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
    q_kdl = kdl.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl

def main():
    import sys
    def usage():
        print("Tests for kdl_parser:\n")
        print("kdl_parser <urdf file>")
        print("\tLoad the URDF from file.")
        print("kdl_parser")
        print("\tLoad the URDF from the parameter server.")
        sys.exit(1)

    if len(sys.argv) > 2:
        usage()
    if len(sys.argv) == 2 and (sys.argv[1] == "-h" or sys.argv[1] == "--help"):
        usage()
    if (len(sys.argv) == 1):
        robot = URDF.load_from_parameter_server(verbose=False)
    else:
        robot = URDF.load_xml_file(sys.argv[1], verbose=False)

    if True:
        import random
        base_link = robot.get_root()
        end_link = robot.links.keys()[random.randint(0, len(robot.links)-1)]
        print "Root link: %s; Random end link: %s" % (base_link, end_link)
        kdl_kin = KDLKinematics(robot, base_link, end_link)
        q = kdl_kin.random_joint_angles()
        print "Random angles:", q
        pose = kdl_kin.forward(q)
        print "FK:", pose
        q_new = kdl_kin.inverse(pose)
        print "IK (not necessarily the same):", q_new
        if q_new is not None:
            pose_new = kdl_kin.forward(q_new)
            print "FK on IK:", pose_new
            print "Error:", np.linalg.norm(pose_new * pose**-1 - np.mat(np.eye(4)))
        else:
            print "IK failure"
        J = kdl_kin.jacobian(q)
        print "Jacobian:", J
        M = kdl_kin.inertia(q)
        print "Inertia matrix:", M
        if False:
            M_cart = kdl_kin.cart_inertia(q)
            print "Cartesian inertia matrix:", M_cart

    if True:
        rospy.init_node("kdl_kinematics")
        num_times = 20
        while not rospy.is_shutdown() and num_times > 0:
            base_link = robot.get_root()
            end_link = robot.links.keys()[random.randint(0, len(robot.links)-1)]
            print "Root link: %s; Random end link: %s" % (base_link, end_link)
            kdl_kin = KDLKinematics(robot, base_link, end_link)
            q = kdl_kin.random_joint_angles()
            pose = kdl_kin.forward(q)
            q_guess = kdl_kin.random_joint_angles()
            q_new = kdl_kin.inverse(pose, q_guess)
            if q_new is None:
                print "Bad IK, trying search..."
                q_search = kdl_kin.inverse_search(pose)
                pose_search = kdl_kin.forward(q_search)
                print "Result error:", np.linalg.norm(pose_search * pose**-1 - np.mat(np.eye(4)))
            num_times -= 1

if __name__ == "__main__":
    main()
