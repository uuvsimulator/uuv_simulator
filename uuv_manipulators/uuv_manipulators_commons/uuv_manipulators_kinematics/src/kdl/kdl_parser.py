#!/usr/bin/env python
#
# A parser for converting Python URDF objects into KDL Trees.
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
# Kyle Maroney

import numpy as np

import rospy

import PyKDL as kdl

from urdf_parser_py.urdf import URDF

def euler_to_quat(r, p, y):
    sr, sp, sy = np.sin(r/2.0), np.sin(p/2.0), np.sin(y/2.0)
    cr, cp, cy = np.cos(r/2.0), np.cos(p/2.0), np.cos(y/2.0)
    return [sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy]

def urdf_pose_to_kdl_frame(pose):
    pos = [0., 0., 0.]
    rot = [0., 0., 0.]
    if pose is not None:
        if pose.position is not None:
            pos = pose.position
        if pose.rotation is not None:
            rot = pose.rotation
    return kdl.Frame(kdl.Rotation.Quaternion(*euler_to_quat(*rot)),
                     kdl.Vector(*pos))

def urdf_joint_to_kdl_joint(jnt):
    origin_frame = urdf_pose_to_kdl_frame(jnt.origin)
    if jnt.joint_type == 'fixed':
        return kdl.Joint(jnt.name, kdl.Joint.None)
    axis = kdl.Vector(*[float(s) for s in jnt.axis])
    if jnt.joint_type == 'revolute':
        return kdl.Joint(jnt.name, origin_frame.p,
                         origin_frame.M * axis, kdl.Joint.RotAxis)
    if jnt.joint_type == 'continuous':
        return kdl.Joint(jnt.name, origin_frame.p,
                         origin_frame.M * axis, kdl.Joint.RotAxis)
    if jnt.joint_type == 'prismatic':
        return kdl.Joint(jnt.name, origin_frame.p,
                         origin_frame.M * axis, kdl.Joint.TransAxis)
    print "Unknown joint type: %s." % jnt.joint_type
    return kdl.Joint(jnt.name, kdl.Joint.None)

def urdf_inertial_to_kdl_rbi(i):
    origin = urdf_pose_to_kdl_frame(i.origin)
    rbi = kdl.RigidBodyInertia(i.mass, origin.p,
                               kdl.RotationalInertia(i.inertia.ixx,
                                                     i.inertia.iyy,
                                                     i.inertia.izz,
                                                     i.inertia.ixy,
                                                     i.inertia.ixz,
                                                     i.inertia.iyz))
    return origin.M * rbi

# Returns a PyKDL.Tree generated from a urdf_parser_py.urdf.URDF object.
def kdl_tree_from_urdf_model(urdf):
    root = urdf.get_root()
    tree = kdl.Tree(root)
    def add_children_to_tree(parent):
        if parent in urdf.child_map:
            for joint, child_name in urdf.child_map[parent]:
                for lidx, link in enumerate(urdf.links):
                    if child_name == link.name:
                        child = urdf.links[lidx]
                        if child.inertial is not None:
                            kdl_inert = urdf_inertial_to_kdl_rbi(child.inertial)
                        else:
                            kdl_inert = kdl.RigidBodyInertia()
                        for jidx, jnt in enumerate(urdf.joints):
                            if jnt.name == joint:
                                kdl_jnt = urdf_joint_to_kdl_joint(urdf.joints[jidx])
                                kdl_origin = urdf_pose_to_kdl_frame(urdf.joints[jidx].origin)
                                kdl_sgm = kdl.Segment(child_name, kdl_jnt,
                                                      kdl_origin, kdl_inert)
                                tree.addSegment(kdl_sgm, parent)
                                add_children_to_tree(child_name)
    add_children_to_tree(root)
    return tree

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
    tree = kdl_tree_from_urdf_model(robot)
    num_non_fixed_joints = 0
    for j in robot.joints:
        if robot.joints[j].joint_type != 'fixed':
            num_non_fixed_joints += 1
    print "URDF non-fixed joints: %d;" % num_non_fixed_joints,
    print "KDL joints: %d" % tree.getNrOfJoints()
    print "URDF joints: %d; KDL segments: %d" %(len(robot.joints),
                                                tree.getNrofSegments())
    import random
    base_link = robot.get_root()
    end_link = robot.links.keys()[random.randint(0, len(robot.links)-1)]
    chain = tree.getChain(base_link, end_link)
    print "Root link: %s; Random end link: %s" % (base_link, end_link)
    for i in range(chain.getNrOfSegments()):
        print chain.getSegment(i).getName()

if __name__ == "__main__":
    main()
