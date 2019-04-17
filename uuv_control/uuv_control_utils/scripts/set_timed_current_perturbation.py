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
from numpy import pi
from uuv_world_ros_plugins_msgs.srv import *

if __name__ == '__main__':
    print('Starting current perturbation node')
    rospy.init_node('set_timed_current_perturbation')

    print('Programming the generation of a current perturbation')
    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    starting_time = 0.0
    if rospy.has_param('~starting_time'):
        starting_time = rospy.get_param('~starting_time')
        if starting_time < 0.0:
            print('Negative starting time, setting it to 0.0')
            starting_time = 0.0

    print('Starting time=', starting_time)

    end_time = -1
    if rospy.has_param('~end_time'):
        end_time = rospy.get_param('~end_time')
        if end_time != -1 and end_time <= starting_time:
            raise rospy.ROSException('End time is smaller than the starting time')

    print('End time=', (end_time if end_time != -1 else 'Inf.'))

    vel = 0.0
    if rospy.has_param('~current_velocity'):
        vel = rospy.get_param('~current_velocity')

    print('Current velocity [m/s]=', vel)

    horz_angle = 0.0
    if rospy.has_param('~horizontal_angle'):
        horz_angle = rospy.get_param('~horizontal_angle')
        horz_angle *= pi / 180

    print('Current horizontal angle [deg]=', horz_angle * 180 / pi)

    vert_angle = 0.0
    if rospy.has_param('~vertical_angle'):
        vert_angle = rospy.get_param('~vertical_angle')
        vert_angle *= pi / 180

    print('Current vertical angle [deg]=', horz_angle * 180 / pi)

    try:
        rospy.wait_for_service('/hydrodynamics/set_current_velocity', timeout=20)
    except rospy.ROSException:
        print('Current velocity services not available! Closing node...')
        sys.exit(-1)

    try:
        set_current = rospy.ServiceProxy('/hydrodynamics/set_current_velocity', SetCurrentVelocity)
    except rospy.ServiceException as e:
        print('Service call failed, error=', e)
        sys.exit(-1)

    # Wait to set the current model
    rate = rospy.Rate(100)
    if rospy.get_time() < starting_time:
        while rospy.get_time() < starting_time:
            rate.sleep()

    print('Applying current model...')
    if set_current(vel, horz_angle, vert_angle):
        print('Current velocity changed successfully at %f s! vel= %f m/s' % (rospy.get_time(), vel))
    else:
        print('Failed to change current velocity')

    if end_time != -1:
        while rospy.get_time() < end_time:
            rate.sleep()

        print('TIMEOUT, setting current velocity to zero...')
        set_current(0, horz_angle, vert_angle)

    print('Leaving node...')
