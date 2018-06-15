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


if __name__ == '__main__':
    rospy.init_node('set_simulation_timer')

    if rospy.is_shutdown():
        rospy.ROSException('ROS master is not running!')

    timeout = 0.0
    if rospy.has_param('~timeout'):
        timeout = rospy.get_param('~timeout')
        if timeout <= 0:
            raise rospy.ROSException('Termination time must be a positive floating point value')

    print 'Starting simulation timer - Timeout = %2.f s' % timeout
    rate = rospy.Rate(100)
    while rospy.get_time() < timeout:
        rate.sleep()

    print 'Simulation timeout - Killing simulation...'
