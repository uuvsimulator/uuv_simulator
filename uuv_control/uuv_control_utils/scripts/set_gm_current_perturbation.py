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
    rospy.init_node('set_gm_current_perturbation')

    print('Programming the generation of a current perturbation')
    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    params = ['component', 'mean', 'min', 'max', 'noise', 'mu']
    values = dict()
    for p in params:
        assert rospy.has_param('~' + p)
        values[p] = rospy.get_param('~' + p)

    assert values['component'] in ['velocity', 'horz_angle', 'vert_angle']
    if values['component'] == 'velocity':
        assert values['mean'] > 0
    else:
        values['min'] *= pi / 180.0
        values['max'] *= pi / 180.0
        values['mean'] *= pi / 180.0

    assert values['min'] < values['max']
    assert values['noise'] >= 0
    assert values['mu'] >= 0

    rospy.wait_for_service(
        '/hydrodynamics/set_current_%s_model' % values['component'],
        timeout=30)

    set_model = rospy.ServiceProxy(
        '/hydrodynamics/set_current_%s_model' % values['component'],
        SetCurrentModel)

    if set_model(values['mean'], values['min'], values['max'], values['noise'],
                 values['mu']):
        print('Model for <{}> set successfully!'.format(values['component']))
    else:
        print('Error setting model!')
