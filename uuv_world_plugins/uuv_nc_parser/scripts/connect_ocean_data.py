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
import rostopic
import rosgraph
import message_filters

from uuv_nc_parser import NCParser
from os.path import isfile
import sys
import numpy as np
from threading import Lock
from copy import deepcopy
from tf.transformations import quaternion_matrix, quaternion_about_axis
from uuv_nc_parser.srv import *
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from geometry_msgs.msg import Vector3, Point32
from sensor_msgs.msg import Temperature, PointCloud, ChannelFloat32
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelProperties

class OceanDataPlayBack:
    def __init__(self):
        q = quaternion_about_axis(np.pi, (1, 0, 0))
        self._toNEDrot = quaternion_matrix(q)[0:3, 0:3]
        self._lock = Lock()
        self._last_vehicle_pos = rospy.Time(0)

        self._filename = ''
        if rospy.has_param('~filename'):
            self._filename = rospy.get_param('~filename')

            if rospy.has_param('~labels'):
                labels = rospy.get_param('~labels')
            else:
                labels = None

            self._ocean_data = NCParser(self._filename, labels)
            print self._ocean_data
        else:
            raise rospy.ROSException('NC data filename not available')

        if rospy.has_param('~fixed_time'):
            self._fixed_time = rospy.get_param('~fixed_time')
            print 'Using fixed time=', self._fixed_time
        else:
            self._fixed_time = None

        if rospy.has_param('~loop_time'):
            self._loop_time = rospy.get_param('~loop_time')
        else:
            self._loop_time = False

        if rospy.has_param('~x_offset'):
            self._x_offset = rospy.get_param('~x_offset')
        else:
            self._x_offset = 0.0

        if rospy.has_param('~center_x'):
            if rospy.get_param('~center_x'):
                self._x_offset = self._ocean_data.min_x + (self._ocean_data.max_x - self._ocean_data.min_x) / 2.0

        if rospy.has_param('~y_offset'):
            self._y_offset = rospy.get_param('~y_offset')
        else:
            self._y_offset = 0.0

        if rospy.has_param('~center_y'):
            if rospy.get_param('~center_y'):
                self._y_offset = self._ocean_data.min_y + (self._ocean_data.max_y - self._ocean_data.min_y) / 2.0

        if rospy.has_param('~variables'):
            self._variables = rospy.get_param('~variables')
        else:
            self._variables = None

        self._max_pnts = 300
        if rospy.has_param('~max_size_pc'):
            if rospy.get_param('~max_size_pc'):
                self._max_pnts = rospy.get_param('~max_size_pc')

        self._time_offset = 0
        if rospy.has_param('~time_offset'):
            offset = rospy.get_param('~time_offset')
            if 'day' in offset:
                self._time_offset += int(offset['day']) * self._ocean_data.DAY2HOUR * self._ocean_data.HOUR2MIN * self._ocean_data.MIN2SEC
            if 'hour' in offset:
                self._time_offset += int(offset['hour']) * self._ocean_data.HOUR2MIN * self._ocean_data.MIN2SEC
            if 'min' in offset:
                self._time_offset += int(offset['min']) * self._ocean_data.MIN2SEC
            print 'Starting time: %d days, %d hours, %d minutes' % (int(offset['day']),
                                                                    int(offset['hour']),
                                                                    int(offset['min']))
            print 'Starting time in seconds:', self._time_offset

        self._services = dict()

        self._services['interpolate_nc_data'] = rospy.Service(
            'interpolate_nc_data', GetNCData, self.get_nc_data)

        self._services['get_nc_variables'] = rospy.Service(
            'get_nc_variables', GetNCVariables, self.get_nc_variables)

        self._services['get_nc_wind_velocity'] = rospy.Service(
            'get_nc_wind_velocity', GetNCVelocity, self.get_nc_wind_velocity)

        self._services['get_nc_current_velocity'] = rospy.Service(
            'get_nc_current_velocity', GetNCVelocity,
            self.get_nc_current_velocity)

        ###############################################################
        # Initializing topics for each vehicle
        ###############################################################

        # Table of vehicle positions
        self._vehicle_pos = dict()
        # Individual topics for each interpolated data
        self._topics = dict()
        # List of static objects to be ignored
        self._static_objs = list()
        # Generate point clouds with all the sampled point variables
        self._pc_variables = dict()
        self._pc_msgs = dict()

        for var in self._variables:
            self._pc_variables[var] = rospy.Publisher('/ocean_data/%s' % var,
                                                      PointCloud,
                                                      queue_size=1)
            self._pc_msgs[var] = PointCloud()
            self._pc_msgs[var].header.frame_id = 'world'
            self._pc_msgs[var].channels.append(ChannelFloat32())

        try:
            rospy.wait_for_service('/gazebo/get_model_properties', timeout=30)
        except rospy.ROSException:
            print 'Service not available! Closing node...'
            sys.exit(-1)

        try:
            self._model_prop_srv = rospy.ServiceProxy(
                '/gazebo/get_model_properties', GetModelProperties)
        except rospy.ServiceException, e:
            print 'Service call failed, error=', e
            sys.exit(-1)

        self._sub_gazebo_models = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.update_vehicle_pos)

        self._update_rate = 50.0
        if rospy.has_param('~update_rate'):
            rate = rospy.get_param('~update_rate')
            if rate > 0:
                self._update_rate = float(rate)
            else:
                print 'Invalid update rate, keeping default of 50 Hz'

        if rospy.has_param('~current_velocity'):
            self._current_vel_config = rospy.get_param('~current_velocity')
            self._timer_current = rospy.Timer(rospy.Duration(1 / self._update_rate),
                                              self.publish_current_velocity)
            print 'Publishing local current velocity'
        else:
            self._current_vel_config = None

        if rospy.has_param('~wind_velocity'):
            self._wind_vel_config = rospy.get_param('~wind_velocity')
            self._timer_wind = rospy.Timer(rospy.Duration(1 / self._update_rate),
                                           self.publish_wind_velocity)
            print 'Publishing local wind velocity'
        else:
            self._wind_vel_config = None

        self._timer_var = rospy.Timer(rospy.Duration(1 / self._update_rate),
                                      self.publish_variables)

    def update_vehicle_pos(self, msg):
        now = rospy.get_rostime()
        dt = now - self._last_vehicle_pos
        if dt.to_sec() < 1 / self._update_rate:
            return

        with self._lock:
            # Removing vehicles that have been removed
            remove_names = list()
            for name in self._vehicle_pos:
                if name not in msg.name:
                    remove_names.append(name)

            if len(remove_names):
                for name in remove_names:
                    del self._vehicle_pos[name]
                    del self._topics[name]

            # Checking in any new objects are available
            new_vehicles = list()
            for name in msg.name:
                if name not in self._vehicle_pos and name not in self._static_objs:
                    new_vehicles.append(name)

            if len(new_vehicles) > 0:
                for name in new_vehicles:
                    resp = self._model_prop_srv(name)
                    if not resp.is_static and rospy.has_param('/%s/robot_description' % name):
                        print 'NEW VEHICLE DETECTED:', name
                        self._vehicle_pos[name] = np.zeros(3)
                        # Create the topics for the new vehicle
                        self._topics[name] = dict()

                        self._topics[name]['current_velocity'] = rospy.Publisher('/%s/current_velocity' % name, Vector3, queue_size=1)
                        self._topics[name]['wind_velocity'] = rospy.Publisher('/%s/wind_velocity' % name, Vector3, queue_size=1)

                        for var in self._variables:
                            if 'temperature' in var.lower():
                                self._topics[name][var] = rospy.Publisher('/%s/%s' % (name, var), Temperature, queue_size=1)
                            else:
                                self._topics[name][var] = rospy.Publisher('/%s/%s' % (name, var), FloatStamped, queue_size=1)
                    else:
                        print 'Static object found:', name
                        self._static_objs.append(name)

            # Updating the position of the non-static objects in the simulation
            for name in self._vehicle_pos:
                for i in range(len(msg.name)):
                    if name == msg.name[i]:
                        self._vehicle_pos[name] = np.array([msg.pose[i].position.x,
                                                            msg.pose[i].position.y,
                                                            msg.pose[i].position.z])
                        break
            self._last_vehicle_pos = now

    def publish_wind_velocity(self, event):
        if self._wind_vel_config is None:
            return True

        t = rospy.get_time()
        with self._lock:
            for name in self._vehicle_pos:
                w_east = self._interpolate(
                    self._wind_vel_config['w_east'],
                    self._vehicle_pos[name][0],
                    self._vehicle_pos[name][1],
                    self._vehicle_pos[name][2],
                    t)
                w_north = self._interpolate(
                    self._wind_vel_config['w_north'],
                    self._vehicle_pos[name][0],
                    self._vehicle_pos[name][1],
                    self._vehicle_pos[name][2],
                    t)
                nedVel = np.array([w_north, w_east, 0])
                enuVel = np.dot(self._toNEDrot.T, nedVel)
                output = Vector3(*enuVel)
                self._topics[name]['wind_velocity'].publish(output)
        return True

    def publish_current_velocity(self, event):
        if self._current_vel_config is None:
            return True

        t = rospy.get_time()
        with self._lock:
            for name in self._vehicle_pos:
                u_east = self._interpolate(
                    self._current_vel_config['u_east'],
                    self._vehicle_pos[name][0],
                    self._vehicle_pos[name][1],
                    self._vehicle_pos[name][2],
                    t)
                v_north = self._interpolate(
                    self._current_vel_config['v_north'],
                    self._vehicle_pos[name][0],
                    self._vehicle_pos[name][1],
                    self._vehicle_pos[name][2],
                    t)

                nedVel = np.array([v_north, u_east, 0])
                enuVel = np.dot(self._toNEDrot.T, nedVel)
                output = Vector3(*enuVel)
                self._topics[name]['current_velocity'].publish(output)
        return True

    def publish_variables(self, event):
        with self._lock:
            t = rospy.get_time()
            for var in self._variables:
                self._pc_msgs[var].header.stamp = rospy.Time.now()
                for name in self._vehicle_pos:
                    value = self._interpolate(
                        var,
                        self._vehicle_pos[name][0],
                        self._vehicle_pos[name][1],
                        self._vehicle_pos[name][2],
                        t)
                    # Updating the point cloud for this variable
                    self._pc_msgs[var].points.append(Point32(self._vehicle_pos[name][0],
                                                             self._vehicle_pos[name][1],
                                                             self._vehicle_pos[name][2]))
                    self._pc_msgs[var].channels[0].name = 'intensity'
                    self._pc_msgs[var].channels[0].values.append(value)

                    if len(self._pc_msgs[var].points) >= self._max_pnts:
                        self._pc_msgs[var].points = self._pc_msgs[var].points[1::]
                        self._pc_msgs[var].channels[0].values = self._pc_msgs[var].channels[0].values[1::]

                    # Create the message objects
                    if 'temperature' in var.lower():
                        msg = Temperature()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = '%s/base_link' % name
                        # TODO Read from the unit of temperature from NC file
                        # Converting to Celsius
                        msg.temperature = value - 273.15
                    else:
                        msg = FloatStamped()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = '%s/base_link' % name
                        msg.data = value
                    self._topics[name][var].publish(msg)
                self._pc_variables[var].publish(self._pc_msgs[var])
        return True

    def _get_all_variables(self):
        var = list()
        options = deepcopy(self._variables)
        if self._current_vel_config is not None:
            options += self._current_vel_config.values()
        if self._wind_vel_config is not None:
            options += self._wind_vel_config.values()

        for v in options:
            if v in self._ocean_data.variable_names:
                var.append(v)
            else:
                print '%s not a valid variable' % v
        return var

    def get_nc_wind_velocity(self, request):
        output = Vector3(0, 0, 0)
        if self._wind_vel_config is not None:
            w_east = self._interpolate(
                self._wind_vel_config['w_east'],
                request.x,
                request.y,
                request.z,
                request.time)
            w_north = self._interpolate(
                self._wind_vel_config['w_north'],
                request.x,
                request.y,
                request.z,
                request.time)

            nedVel = np.array([w_north, w_east, 0])
            enuVel = np.dot(self._toNEDrot.T, nedVel)
            output = Vector3(*enuVel)
        return GetNCVelocityResponse(output)

    def get_nc_current_velocity(self, request):
        output = Vector3(0, 0, 0)
        if self._current_vel_config is not None:
            u_east = self._interpolate(
                self._current_vel_config['u_east'],
                request.x,
                request.y,
                request.z,
                request.time)
            v_north = self._interpolate(
                self._current_vel_config['v_north'],
                request.x,
                request.y,
                request.z,
                request.time)

            nedVel = np.array([v_north, u_east, 0])
            enuVel = np.dot(self._toNEDrot.T, nedVel)
            output = Vector3(*enuVel)
        return GetNCVelocityResponse(output)

    def get_nc_variables(self, request):
        return GetNCVariablesResponse(self._variables)

    def get_nc_data(self, request):
        if request.variable not in self._get_all_variables():
            print 'Invalid variable, var_name=', request.variable
            return GetNCDataResponse(0.0)

        return GetNCDataResponse(
            self._interpolate(request.variable,
                              request.x,
                              request.y,
                              request.z,
                              request.time))

    def _interpolate(self, variable, x, y, z, time):
        ENUpos = np.array([x, y, z])
        # Since the odometry given by Gazebo is in ENU standard, transform into
        # NED to interpolate on the ocean data
        pos = np.dot(self._toNEDrot, ENUpos)
        # Add x and y offsets
        x = pos[0] + self._x_offset
        y = pos[1] + self._y_offset
        z = pos[2]

        if not self._loop_time:
            # Use fixed or simulation time
            t = (time if self._fixed_time is None else self._fixed_time)
        else:
            # Loop the time vector, if needed
            t = time % self._ocean_data.end_time
        t += float(self._time_offset)

        # Interpolate the given variables on the current position and time
        output = self._ocean_data.interpolate(
            variable, x, y, z, t)
        return output

if __name__ == '__main__':
    print 'Ocean data playback'
    rospy.init_node('connect_to_ocean_data')

    try:
        pb = OceanDataPlayBack()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
