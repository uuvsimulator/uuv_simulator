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
import sys
from uuv_control_msgs.srv import *
from std_msgs.msg import String, Time


if __name__ == '__main__':
    print 'Send a waypoint file, namespace=', rospy.get_namespace()
    rospy.init_node('send_waypoint_file')

    if rospy.is_shutdown():
        print 'ROS master not running!'
        sys.exit(-1)

    if rospy.has_param('~filename'):
        filename = rospy.get_param('~filename')
    else:
        raise rospy.ROSException('No filename found')

    # If no start time is provided: start *now*.
    start_time = rospy.Time.now().to_sec()
    start_now = True
    if rospy.has_param('~start_time'):
        start_time = rospy.get_param('~start_time')
        if start_time < 0.0:
            print 'Negative start time, setting it to 0.0'
            start_time = 0.0
            start_now = True
    else:
        start_now = True

    try:
        rospy.wait_for_service('init_waypoints_from_file', timeout=2)
    except rospy.ROSException:
        rospy.ROSException('Service not available! Closing node...')

    try:
        init_wp = rospy.ServiceProxy(
            'init_waypoints_from_file',
            InitWaypointsFromFile)
    except rospy.ServiceException, e:
        rospy.ROSException('Service call failed, error=' + e)

    success = init_wp(Time(rospy.Time(start_time)),
                      start_now,
                      String(filename))

    if success:
        print 'Waypoints file successfully received, file=', filename
    else:
        print 'Failed'
