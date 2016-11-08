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
from os.path import isdir, join
from copy import deepcopy
import yaml

from uuv_thrusters import ThrusterManager

from geometry_msgs.msg import Wrench

from uuv_thruster_manager.srv import *


class ThrusterAllocatorNode(ThrusterManager):
    """
    The thruster allocator node allows a client node to command the thrusters.
    """

    def __init__(self):
        """Class constructor."""
        ThrusterManager.__init__(self)

        self.last_update = rospy.Time.now()

        # Subscriber to the wrench to be applied on the UUV
        self.input_sub = rospy.Subscriber('thruster_manager/input',
                                          Wrench, self.input_callback)

        self.thruster_info_service = rospy.Service(
            'thruster_manager/get_thrusters_info', ThrusterManagerInfo,
            self.get_thruster_info)
        self.curve_calc_service = rospy.Service(
            'thruster_manager/get_thruster_curve', GetThrusterCurve,
            self.get_thruster_curve)
        self.set_thruster_manager_config_service = rospy.Service(
            'thruster_manager/set_config', SetThrusterManagerConfig,
            self.set_config)
        self.get_thruster_manager_config_service = rospy.Service(
            'thruster_manager/get_config', GetThrusterManagerConfig,
            self.get_config)

        rate = rospy.Rate(self.config['update_rate'])
        while not rospy.is_shutdown():
            if self.config['timeout'] > 0:
                # If a timeout is set, zero the outputs to the thrusters if
                # there is no command signal for the length of timeout
                if rospy.Time.now() - self.last_update > self.config['timeout']:
                    print 'Turning thrusters off - inactive for too long'
                    if self.thrust is not None:
                        self.thrust.fill(0)
                        self.command_thrusters()
            rate.sleep()

    def get_thruster_info(self, request):
        """Return service callback with thruster information."""
        return ThrusterManagerInfoResponse(
            self.n_thrusters,
            self.configuration_matrix.flatten().tolist(),
            self.namespace + self.config['base_link'])

    def get_thruster_curve(self, request):
        """Return service callback for computation of thruster curve."""
        if self.n_thrusters == 0:
            return GetThrusterCurveResponse([], [])
        # TODO Get thruster index, for the case the vehicle has different
        # models
        input_values, thrust_values = self.thrusters[0].get_curve(
            request.min, request.max, request.n_points)
        return GetThrusterCurveResponse(input_values, thrust_values)

    def set_config(self, request):
        old_config = deepcopy(self.config)
        self.ready = False
        self.config['base_link'] = request.base_link
        self.config['thruster_frame_base'] = request.thruster_frame_base
        self.config['thruster_topic_prefix'] = request.thruster_topic_prefix
        self.config['thruster_topic_suffix'] = request.thruster_topic_suffix
        self.config['timeout'] = request.timeout
        print 'New configuration:\n'
        for key in self.config:
            print key, '=', self.config[key]
        if not self.update_tam(recalculate=True):
            print 'Configuration parameters are invalid, going back to old configuration...'
            self.config = old_config
            self.update_tam(recalculate=True)
        return SetThrusterManagerConfigResponse(True)

    def get_config(self, request):
        return GetThrusterManagerConfigResponse(
            self.namespace,
            self.config['base_link'],
            self.config['thruster_frame_base'],
            self.config['thruster_topic_prefix'],
            self.config['thruster_topic_suffix'],
            self.config['timeout'],
            self.config['max_thrust'],
            self.n_thrusters,
            self.configuration_matrix.flatten().tolist())

    def input_callback(self, msg):
        """
        Callback to the subscriber that receiver the wrench to be applied on
        UUV's BODY frame.
        @param msg Wrench message
        """
        if not self.ready:
            return

        force = numpy.array((msg.force.x, msg.force.y, msg.force.z))
        torque = numpy.array((msg.torque.x, msg.torque.y, msg.torque.z))

        self.publish_thrust_forces(force, torque)

        self.last_update = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('thruster_allocator')

    try:
        node = ThrusterAllocatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print 'ThrusterAllocatorNode::Exception'
    print 'Leaving ThrusterAllocatorNode'
