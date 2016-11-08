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

import numpy
import rospy
import tf
import tf.transformations as trans
import sys

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, JointState
from rospy.numpy_msg import numpy_msg

import argparse


class RampGenerationNode:
    def __init__(self, namespace, arm, slope):
        assert slope > 0, 'Slope cannot be negative'
        assert len(namespace) > 0, 'Namespace string is empty'

        self.sub = {}
        self.pub = {}

        self.ramp = {}
        self.slope = {}
        # Stores the last time stamp for the call for a ramp generation
        self.last_activation = {}
        # Joint limits
        self.joint_min = {}
        self.joint_max = {}
        # Starting time
        self.start_time = rospy.get_time()

        # Storing the robot's namespace
        self.namespace = namespace
        # Slope rate for the ramp
        self.slope_rate = slope

        if self.namespace[0] != '/':
            self.namespace = '/' + self.namespace
        if self.namespace[-1] != '/':
            self.namespace = self.namespace + '/'

        activeJointLimits = rospy.get_param(self.namespace + 'arms/' + arm + '/joint_limits')

        assert activeJointLimits is not None, 'Joint limits have not been loaded to the parameter server'

        self.jointNames = activeJointLimits.keys()
        self.jointPos = {}

        print 'RampGenerationNode::namespace=', self.namespace
        print 'RampGenerationNode::joints=', self.jointNames

        self.jointStateSub = rospy.Subscriber(self.namespace + 'joint_states', JointState, self.joint_state_callback)

        for joint in activeJointLimits:
            self.pub[joint] = rospy.Publisher(self.namespace + arm + '/' + joint + '/controller/command', Float64, queue_size=10)
            self.sub[joint] = rospy.Subscriber(self.namespace + arm + '/' + joint + '/ramp_generator/command', numpy_msg(Float64), self.ramp_callback, (joint))
            # Set initial joint angle
            self.ramp[joint] = Float64()
            self.ramp[joint].data = 0.0
            # Get joint limits
            self.joint_min[joint] = activeJointLimits[joint]['min']
            self.joint_max[joint] = activeJointLimits[joint]['max']

            assert self.joint_min[joint] < self.joint_max[joint], 'Invalid joint limit, joint=' + joint

            self.jointPos[joint] = None
            self.slope[joint] = 0.0
            self.last_activation[joint] = self.start_time

        rate = rospy.Rate(500)
        jointPosInit = False
        print 'RampGenerationNode::start'
        while not rospy.is_shutdown():
            # Read initial position of each joint
            for joint in self.jointNames:
                if self.jointPos[joint] is None and not jointPosInit:
                    # print 'RampGenerationNode - No joint position available'
                    continue
                elif self.jointPos[joint] is not None and not jointPosInit:
                    self.ramp[joint].data = self.jointPos[joint]
                    jointPosInit = True

                # If the control input has been deactivated for too long, set
                # slope to zero
                if rospy.get_time() - self.last_activation[joint] >= 0.5:
                    self.slope[joint] = 0.0

                self.ramp[joint].data = self.slope[joint] * rospy.get_time() + self.ramp[joint].data

                if self.ramp[joint].data < self.joint_min[joint]:
                    self.ramp[joint].data = self.joint_min[joint]
                elif self.ramp[joint].data > self.joint_max[joint]:
                    self.ramp[joint].data = self.joint_max[joint]
                self.pub[joint].publish(self.ramp[joint])
            rate.sleep()
        print 'RampGenerationNode::end'

    def ramp_callback(self, msg, joint):
        if rospy.get_time() > self.last_activation[joint]:
            self.slope[joint] = msg.data * self.slope_rate
            self.last_activation[joint] = rospy.get_time()

    def joint_state_callback(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            joint = name.split('/')[-1]
            self.jointPos[joint] = float(msg.position[i])

if __name__ == '__main__':
    print 'Starting the RampGenerationNode'

    # Create argument parser
    parser = argparse.ArgumentParser(description='Cartesian Teleoperation Node::XBox360')
    parser.add_argument('--namespace', metavar='NS', type=str)
    parser.add_argument('--arm', metavar='ARM', type=str)
    parser.add_argument('--slope', metavar='SLOPE', type=float)
    # Filter out the garbage
    arguments = [a for a in sys.argv if ':=' not in a and '.py' not in a]

    args = parser.parse_args(arguments)

    namespace = ''

    if args.namespace:
        namespace = args.namespace

    slope = 1 / 1e5

    if args.slope:
        slope = args.slope

    assert args.arm is not None, 'Name of the arm to be controller is missing'

    arm = args.arm

    rospy.init_node('joy_ramp_generation')

    try:
        node = RampGenerationNode(namespace, arm, slope)
        rospy.spin()
    except rospy.ROSInterruptException:
        print 'RampGenerationNode::Exception'
    print 'Leaving RampGenerationNode'
