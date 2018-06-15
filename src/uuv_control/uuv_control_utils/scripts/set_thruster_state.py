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

import rospy
from uuv_gazebo_ros_plugins_msgs.srv import SetThrusterState


if __name__ == '__main__':
    print 'Set the state of thrusters for vehicle, namespace=', rospy.get_namespace()
    rospy.init_node('set_thrusters_states')

    if rospy.is_shutdown():
        rospy.ROSException('ROS master not running!')

    starting_time = 0.0
    if rospy.has_param('~starting_time'):
        starting_time = rospy.get_param('~starting_time')

    print 'Starting time= %fs' % starting_time

    duration = 0.0
    if rospy.has_param('~duration'):
        duration = rospy.get_param('~duration')

    if duration == 0.0:
        rospy.ROSException('Duration not set, leaving node...')

    print 'Duration [s]=', ('Inf.' if duration < 0 else duration)

    if rospy.has_param('~is_on'):
        is_on = bool(rospy.get_param('~is_on'))
    else:
        rospy.ROSException('State flag not provided')

    if rospy.has_param('~thruster_id'):
        thruster_id = rospy.get_param('~thruster_id')
    else:
        rospy.ROSException('Thruster ID not given')

    if thruster_id < 0:
        rospy.ROSException('Invalid thruster ID')

    print 'Setting state of thruster #%d as %s' % (thruster_id, 'ON' if is_on else 'OFF')

    vehicle_name = rospy.get_namespace().replace('/', '')

    srv_name = '/%s/thrusters/%d/set_thruster_state' % (vehicle_name, thruster_id)

    try:
        rospy.wait_for_service(srv_name, timeout=2)
    except rospy.ROSException:
        rospy.ROSException('Service not available! Closing node...')

    try:
        set_state = rospy.ServiceProxy(srv_name, SetThrusterState)
    except rospy.ServiceException, e:
        rospy.ROSException('Service call failed, error=' + e)

    rate = rospy.Rate(100)
    while rospy.get_time() < starting_time:
        rate.sleep()

    success = set_state(is_on)

    if success:
        print 'Time=%.2f s' % rospy.get_time()
        print 'Current state of thruster #%d=%s' % (thruster_id, 'ON' if is_on else 'OFF')

    if duration > 0:
        rate = rospy.Rate(100)
        while rospy.get_time() < starting_time + duration:
            rate.sleep()

        success = set_state(not is_on)

        if success:
            print 'Time=%.2f s' % rospy.get_time()
            print 'Returning to previous state of thruster #%d=%s' % (thruster_id, 'ON' if not is_on else 'OFF')

    print 'Leaving node...'
