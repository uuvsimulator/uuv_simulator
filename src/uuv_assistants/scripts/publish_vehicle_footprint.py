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
from copy import deepcopy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
import numpy as np


class VehicleFootprint:
    MARKER = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])

    def __init__(self):
        self._namespace = rospy.get_namespace().replace('/', '')

        self._scale_footprint = 10

        if rospy.has_param('~scale_footprint'):
            scale = rospy.get_param('~scale_footprint')
            if scale > 0:
                self._scale_footprint = scale
            else:
                print 'Scale factor should be greater than zero'
        
        print 'Footprint marker scale factor = ', self._scale_footprint

        self._scale_label = 10

        if rospy.has_param('~_scale_label'):
            scale = rospy.get_param('~_scale_label')
            if scale > 0:
                self._scale_label = scale
            else:
                print 'Scale factor should be greater than zero'
        
        print 'Label marker scale factor = ', self._scale_label

        self._label_x_offset = 60
        if rospy.get_param('~label_x_offset'):
            self._label_x_offset = rospy.get_param('~label_x_offset')
            
        self._label_marker = Marker()
        self._label_marker.header.frame_id = 'world'
        self._label_marker.header.stamp = rospy.Time.now()
        self._label_marker.ns = self._namespace
        self._label_marker.type = Marker.TEXT_VIEW_FACING
        self._label_marker.text = self._namespace
        self._label_marker.action = Marker.ADD
        self._label_marker.pose.orientation.x = 0.0
        self._label_marker.pose.orientation.y = 0.0
        self._label_marker.pose.orientation.z = 0.0
        self._label_marker.pose.orientation.w = 1.0
        self._label_marker.scale.x = 0.0
        self._label_marker.scale.y = 0.0
        self._label_marker.scale.z = self._scale_label
        self._label_marker.color.a = 1.0
        self._label_marker.color.r = 0.0
        self._label_marker.color.g = 1.0
        self._label_marker.color.b = 0.0

        # Odometry subscriber (remap this topic in the launch file if necessary)
        self._odom_sub = rospy.Subscriber('odom', Odometry, self.odometry_callback)
        # Footprint marker publisher (remap this topic in the launch file if necessary)
        self._footprint_pub = rospy.Publisher('footprint', PolygonStamped, queue_size=1)
        # Vehicle label marker (remap this topic in the launch file if necessary)
        self._label_pub = rospy.Publisher('label', Marker, queue_size=1)
    
    @staticmethod
    def rot(alpha):
        return np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha), np.cos(alpha)]])

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        orientation = np.array([msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w])

        yaw = euler_from_quaternion(orientation)[2]

        # Generating the vehicle footprint marker
        new_marker = self._scale_footprint * deepcopy(self.MARKER)

        points = list()
        for i in range(new_marker.shape[0]):
            new_marker[i, :] = np.dot(self.rot(yaw - np.pi / 2), new_marker[i, :])
            new_marker[i, 0] += x
            new_marker[i, 1] += y

            p = Point32()
            p.x = new_marker[i, 0]
            p.y = new_marker[i, 1]
            points.append(p)

        new_poly = PolygonStamped()
        new_poly.header.stamp = rospy.Time.now()
        new_poly.header.frame_id = 'world'
        new_poly.polygon.points = points

        self._footprint_pub.publish(new_poly)

        # Generating the label marker
        self._label_marker.pose.position.x = msg.pose.pose.position.x + self._label_x_offset
        self._label_marker.pose.position.y = msg.pose.pose.position.y
        self._label_marker.pose.position.z = msg.pose.pose.position.z 

        self._label_pub.publish(self._label_marker)

if __name__ == '__main__':
    print 'Generate RViz footprint and markers for 2D visualization'
    rospy.init_node('generate_vehicle_footprint')

    try:
        node = VehicleFootprint()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')