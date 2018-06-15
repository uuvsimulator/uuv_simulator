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

import argparse
import sys
import rospy
import time
from std_msgs.msg import Float64


if __name__ == '__main__':
    # Create argument parser
    parser = argparse.ArgumentParser(description='uuv_manipulators_control::set_joint_config')
    parser.add_argument('--namespace', metavar='NS', type=str)
    parser.add_argument('--arm', metavar='NS', type=str)
    parser.add_argument('--config', metavar='CONFIG', type=str)

    # Filter out the garbage
    arguments = [a for a in sys.argv if ':=' not in a and '.py' not in a]

    args = parser.parse_args(arguments)

    # Manipulator's namespace
    assert args.namespace is not None, "Manipulator's namespace not given"
    namespace = args.namespace
    if namespace[0] != '/':
        namespace = '/' + namespace
    if namespace[-1] != '/':
        namespace = namespace + '/'

    # Desired configuration (the joint position must be specified in the
    # parameter server)
    assert args.config is not None, "Manipulator's desired configuration not given"
    config = args.config

    assert args.arm is not None, 'Manipulator name must be given'
    arm_name = args.arm

    rospy.init_node('set_joint_config')

    assert not rospy.is_shutdown(), 'ROS was not initialized'

    try:
        # Read the desired joint configuration from the parameter server
        assert rospy.has_param(namespace + 'arms/' + arm_name + '/default_configs/' + config), 'Desired configuration not in the parameter namespace, config=%s' % config

        joint_pos = rospy.get_param(namespace + 'arms/' + arm_name + '/default_configs/' + config)

        print 'Set joint configuration=', joint_pos
        print 'Output topics:'
        pub = {}
        for joint in joint_pos:
            print '     ', joint, '=', namespace + arm_name + '/' + joint + '/position_controller/command'
            pub[joint] = rospy.Publisher(namespace + arm_name + '/' + joint + '/position_controller/command', Float64, queue_size=1)

        start_time = time.clock()
        duration = 5
        rate = rospy.Rate(100.0)
        rospy.loginfo('Publishing command message for %s seconds' % duration)
        while time.clock() <= start_time + duration:
            if rospy.is_shutdown():
                print 'ROS is not running'
                break

            for joint in joint_pos:
                pos_msg = Float64()
                pos_msg.data = joint_pos[joint]
                pub[joint].publish(pos_msg)

            rate.sleep()
        rospy.loginfo('Finishing setting joint configuration')

    except rospy.ROSInterruptException:
        print 'uuv_manipulators_control::set_joint_config::Exception'
    print 'Leaving uuv_manipulators_control::set_joint_config'
