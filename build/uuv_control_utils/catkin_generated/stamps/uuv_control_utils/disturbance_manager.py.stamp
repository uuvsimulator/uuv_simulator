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
import numpy as np
import logging
from uuv_world_ros_plugins_msgs.srv import *
from gazebo_msgs.srv import ApplyBodyWrench
from uuv_gazebo_ros_plugins_msgs.srv import SetThrusterState, SetThrusterEfficiency
from geometry_msgs.msg import Point, WrenchStamped, Vector3


class DisturbanceManager:

    def __init__(self):
        self._logger = logging.getLogger('dp_local_planner')
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        self._logger.addHandler(out_hdlr)
        self._logger.setLevel(logging.INFO)

        # Load disturbances and check for missing information
        specs = dict(current=['starting_time', 'velocity', 'horizontal_angle',
                              'vertical_angle'],
                     wrench=['starting_time', 'duration', 'force', 'torque'],
                     thruster_state=['starting_time', 'thruster_id', 'is_on',
                                     'duration'],
                     propeller_efficiency=['starting_time', 'thruster_id', 'duration',
                                    'efficiency'],
                     thrust_efficiency=['starting_time', 'thruster_id', 'duration',
                                 'efficiency'])

        thruster_ids = list()

        if rospy.has_param('~disturbances'):
            self._disturbances = rospy.get_param('~disturbances')
            self._logger.info(self._disturbances)
            if type(self._disturbances) != list:
                raise rospy.ROSException('Current specifications must be '
                                         'given as a list of dict')
            for i in range(len(self._disturbances)):
                item = self._disturbances[i]
                if type(item) != dict:
                    raise rospy.ROSException('Disturbance description must be'
                                             ' given as a dict')
                if 'type' not in item:
                    raise rospy.ROSException('Type of disturbance not '
                                             'specified')
                if item['type'] not in specs:
                    raise rospy.ROSException(
                        'Invalid type of disturbance, value=%s' % item['type'])

                for spec in specs[item['type']]:
                    if spec not in item:
                        raise rospy.ROSException(
                            'Invalid current model specification, '
                            'missing tag=%s' % spec)

                if item['type'] == 'thruster_state':
                    thruster_ids.append(item['thruster_id'])

                # Create flag to indicate that perturbation has been applied
                self._disturbances[i]['is_applied'] = False
                self._disturbances[i]['ended'] = False
        else:
            raise rospy.ROSException('No disturbance specifications given')

        # List all disturbances to be applied
        for i in range(len(self._disturbances)):
            self._logger.info('Disturbance #%d: %s' % (i, self._disturbances[i]))

        self._body_force = np.zeros(3)
        self._body_torque = np.zeros(3)
        self._body_wrench_msg = WrenchStamped()

        # For body wrench disturbances, publish a topic
        self._wrench_topic = rospy.Publisher(
            'wrench_perturbation', WrenchStamped, queue_size=1)

        vehicle_name = rospy.get_namespace().replace('/', '')

        # Test if services are reachable
        try:
            services = ['/hydrodynamics/set_current_velocity',
                        '/gazebo/apply_body_wrench']
            for item in self._disturbances:
                if item['type'] == 'thruster_state':
                    services.append('/%s/thrusters/%d/set_thruster_state' % (vehicle_name, item['thruster_id']))
                elif item['type'] == 'propeller_efficiency':
                    services.append('/%s/thrusters/%d/set_dynamic_state_efficiency' % (vehicle_name, item['thruster_id']))
                elif item['type'] == 'thrust_efficiency':
                    services.append('/%s/thrusters/%d/set_thrust_force_efficiency' % (vehicle_name, item['thruster_id']))
            for s in services:
                rospy.wait_for_service(s, timeout=10)
        except Exception, e:
            self._logger.error('Some services are not available! message=' + str(e))
            self._logger.error('Closing node...')
            sys.exit(-1)

        # Obtain service proxy
        self._service_cb = dict()
        try:
            self._service_cb['current_velocity'] = rospy.ServiceProxy(
                '/hydrodynamics/set_current_velocity', SetCurrentVelocity)
            self._service_cb['wrench'] = rospy.ServiceProxy(
                '/gazebo/apply_body_wrench', ApplyBodyWrench)
            self._service_cb['thrusters'] = dict()
            for item in self._disturbances:
                if item['type'] == 'thruster_state':
                    if 'state' not in self._service_cb['thrusters']:
                        self._service_cb['thrusters']['state'] = dict()
                    self._service_cb['thrusters']['state'][item['thruster_id']] = rospy.ServiceProxy(
                        '/%s/thrusters/%d/set_thruster_state' % (vehicle_name, item['thruster_id']),
                        SetThrusterState)
                elif item['type'] == 'propeller_efficiency':
                    if 'propeller_efficiency' not in self._service_cb['thrusters']:
                        self._service_cb['thrusters']['propeller_efficiency'] = dict()
                    self._service_cb['thrusters']['propeller_efficiency'][item['thruster_id']] = rospy.ServiceProxy(
                        '/%s/thrusters/%d/set_dynamic_state_efficiency' % (vehicle_name, item['thruster_id']),
                        SetThrusterEfficiency)
                elif item['type'] == 'thrust_efficiency':
                    if 'thrust_efficiency' not in self._service_cb['thrusters']:
                        self._service_cb['thrusters']['thrust_efficiency'] = dict()
                    self._service_cb['thrusters']['thrust_efficiency'][item['thruster_id']] = rospy.ServiceProxy(
                        '/%s/thrusters/%d/set_thrust_force_efficiency' % (vehicle_name, item['thruster_id']),
                        SetThrusterEfficiency)
        except rospy.ServiceException, e:
            self._logger.info('Service call failed, error=%s' % str(e))
            sys.exit(-1)

        self._wrench_timer = rospy.timer.Timer(rospy.Duration(0.1),
                                               self._publish_wrench_disturbance)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            t = rospy.get_time()
            for i in range(len(self._disturbances)):
                d = self._disturbances[i]
                if t > d['starting_time'] and not d['is_applied']:
                    ###########################################################
                    if d['type'] == 'current':
                        self.set_current(d['velocity'], d['horizontal_angle'],
                                         d['vertical_angle'])
                    ###########################################################
                    elif d['type'] == 'wrench':
                        self.set_body_wrench(d['force'],
                                             d['torque'],
                                             -1,
                                             d['starting_time'])
                    ###########################################################
                    elif d['type'] == 'thruster_state':
                        self.set_thruster_state(d['thruster_id'], bool(d['is_on']))
                    ###########################################################
                    elif d['type'] == 'propeller_efficiency':
                        self.set_propeller_efficiency(d['thruster_id'], d['efficiency'])
                    ###########################################################
                    elif d['type'] == 'thrust_efficiency':
                        self.set_thrust_efficiency(d['thruster_id'], d['efficiency'])
                    # Set applied flag to true
                    self._disturbances[i]['is_applied'] = True

                    if 'duration' in d:
                        if d['duration'] == -1:
                            self._disturbances[i]['ended'] = True
                    else:
                        self._disturbances[i]['ended'] = True
                elif d['is_applied'] and 'duration' in d and not d['ended']:
                    if d['duration'] > 0:
                        if rospy.get_time() > d['starting_time'] + d['duration']:
                            ###########################################################
                            if d['type'] == 'current':
                                # Set current to zero
                                self.set_current(0, d['horizontal_angle'],
                                                 d['vertical_angle'])
                            ###########################################################
                            elif d['type'] == 'wrench':
                                # Cancel out force and torque
                                self.set_body_wrench([-1 * d['force'][n] for n in range(3)],
                                                     [-1 * d['torque'][n] for n in range(3)],
                                                     -1,
                                                     rospy.get_time())
                            ###########################################################
                            elif d['type'] == 'thruster_state':
                                self.set_thruster_state(d['thruster_id'], not bool(d['is_on']))
                            ###########################################################
                            elif d['type'] == 'propeller_efficiency':
                                self.set_propeller_efficiency(d['thruster_id'], 1.0)
                            ###########################################################
                            elif d['type'] == 'thrust_efficiency':
                                self.set_thrust_efficiency(d['thruster_id'], 1.0)

                            self._disturbances[i]['ended'] = True
            rate.sleep()

    def _publish_wrench_disturbance(self, event):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = 'world'
        msg.wrench.force = Vector3(*self._body_force)
        msg.wrench.torque = Vector3(*self._body_torque)
        # Publish the applied body wrench
        self._wrench_topic.publish(msg)
        return True

    def set_current(self, velocity, horizontal_angle, vertical_angle):
        self._logger.info('Appying current velocity model...')
        if self._service_cb['current_velocity'](velocity, horizontal_angle, vertical_angle):
            self._logger.info('Current velocity changed successfully at %f s! vel= %f m/s' % (rospy.get_time(), velocity))
        else:
            self._logger.error('Failed to change current velocity')

    def set_body_wrench(self, force, torque, duration, starting_time):
        ns = rospy.get_namespace().replace('/', '')
        body_name = '%s/base_link' % ns

        self._body_force = np.array([self._body_force[i] + force[i] for i in range(3)])
        self._body_torque = np.array([self._body_torque[i] + torque[i] for i in range(3)])

        self._body_wrench_msg = WrenchStamped()
        self._body_wrench_msg.header.stamp = rospy.Time().now()
        self._body_wrench_msg.header.frame_id = 'world'
        self._body_wrench_msg.wrench.force = Vector3(*self._body_force)
        self._body_wrench_msg.wrench.torque = Vector3(*self._body_torque)
        success = self._service_cb['wrench'](
            body_name,
            'world',
            Point(0, 0, 0),
            self._body_wrench_msg.wrench,
            rospy.Time(secs=starting_time),
            rospy.Duration(duration))

        if success:
            self._logger.info('Body wrench perturbation applied!, body_name=%s, t=%.2f s' % (body_name, rospy.get_time()))
        else:
            self._logger.error('Failed to apply body wrench!, body_name=%s, t=%.2f s' % (body_name, rospy.get_time()))

    def set_thruster_state(self, thruster_id, is_on):
        if self._service_cb['thrusters']['state'][thruster_id](is_on):
            self._logger.info('Setting state of thruster #%d, state=%s, t=%.2f s' % (thruster_id, 'ON' if is_on else 'OFF', rospy.get_time()))
        else:
            self._logger.error('Setting state of thruster #%d failed! t=%.2f s' % (thruster_id, rospy.get_time()))

    def set_propeller_efficiency(self, thruster_id, eff):
        if self._service_cb['thrusters']['propeller_efficiency'][thruster_id](eff):
            self._logger.info('Setting propeller efficiency of thruster #%d, eff=%s, t=%.2f s' % (thruster_id, eff, rospy.get_time()))
        else:
            self._logger.error('Setting propeller efficiency of thruster #%d failed! t=%.2f s' % (thruster_id, rospy.get_time()))

    def set_thrust_efficiency(self, thruster_id, eff):
        if self._service_cb['thrusters']['thrust_efficiency'][thruster_id](eff):
            self._logger.info('Setting thrust efficiency of thruster #%d, eff=%s, t=%.2f s' % (thruster_id, eff, rospy.get_time()))
        else:
            self._logger.error('Setting thrust efficiency of thruster #%d failed! t=%.2f s' % (thruster_id, rospy.get_time()))

if __name__ == '__main__':
    print('Starting disturbance manager')
    rospy.init_node('disturbance_manager')

    try:
        node = DisturbanceManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
