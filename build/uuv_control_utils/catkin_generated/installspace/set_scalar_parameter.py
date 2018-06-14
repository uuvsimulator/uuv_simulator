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
from uuv_gazebo_ros_plugins_msgs.srv import SetFloat


if __name__ == '__main__':
    print('Set scalar parameter, namespace=' + rospy.get_namespace())
    rospy.init_node('set_thrusters_states')

    if rospy.is_shutdown():
        rospy.ROSException('ROS master not running!')

    services = ['set_fluid_density', 'set_added_mass_scaling',
                'set_damping_scaling', 'set_volume_scaling',
                'set_volume_offset', 'set_added_mass_offset',
                'set_linear_damping_offset', 'set_nonlinear_damping_offset',
                'set_linear_forward_speed_damping_offset']

    assert rospy.has_param('~service_name')
    assert rospy.has_param('~data')

    service_name = rospy.get_param('~service_name')
    assert service_name in services, 'Possible service names are=' + str(services)

    data = rospy.get_param('~data')

    rospy.wait_for_service(service_name, timeout=30)

    fcn = rospy.ServiceProxy(service_name, SetFloat)
    print('Set scalar parameter, service=%s, data=%.2f' % (service_name, data))
    output = fcn(data)
    if output.success:
        print('Parameter set successfully')
    else:
        print('Error setting scalar parameter')
    print(output.message)
    print('Leaving node...')
