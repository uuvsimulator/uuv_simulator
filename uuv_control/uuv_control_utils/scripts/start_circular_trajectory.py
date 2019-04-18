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
import rospy
import sys
from uuv_control_msgs.srv import InitCircularTrajectory
from numpy import pi
from geometry_msgs.msg import Point
from std_msgs.msg import Time


if __name__ == '__main__':
    print('Starting the circular trajectory creator')
    rospy.init_node('start_circular_trajectory')

    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    # If no start time is provided: start *now*.
    start_time = rospy.Time.now().to_sec()
    start_now = False
    if rospy.has_param('~start_time'):
        start_time = rospy.get_param('~start_time')
        if start_time < 0.0:
            print('Negative start time, setting it to 0.0')
            start_time = 0.0
            start_now = True
    else:
        start_now = True

    param_labels = ['radius', 'center', 'n_points', 'heading_offset',
                    'duration', 'max_forward_speed']
    params = dict()

    for label in param_labels:
        if not rospy.has_param('~' + label):
            print('{} must be provided for the trajectory generation!'.format(label))
            sys.exit(-1)

        params[label] = rospy.get_param('~' + label)

    if len(params['center']) != 3:
        print('Center of circle must have 3 components (x, y, z)')
        sys.exit(-1)

    if params['n_points'] <= 2:
        print('Number of points must be at least 2')
        sys.exit(-1)

    if params['max_forward_speed'] <= 0:
        print('Velocity limit must be positive')
        sys.exit(-1)

    try:
        rospy.wait_for_service('start_circular_trajectory', timeout=20)
    except rospy.ROSException:
        print('Service not available! Closing node...')
        sys.exit(-1)

    try:
        traj_gen = rospy.ServiceProxy('start_circular_trajectory', InitCircularTrajectory)
    except rospy.ServiceException as e:
        print('Service call failed, error={}'.format(e))
        sys.exit(-1)

    print('Generating trajectory that starts at t={} s'.format(start_time))

    success = traj_gen(Time(rospy.Time(start_time)),
                       start_now,
                       params['radius'],
                       Point(params['center'][0], params['center'][1], params['center'][2]),
                       False,
                       0.0,
                       params['n_points'],
                       params['heading_offset'] * pi / 180,
                       params['max_forward_speed'],
                       params['duration'])

    if success:
        print('Trajectory successfully generated!')
    else:
        print('Failed')
