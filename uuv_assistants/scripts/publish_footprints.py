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
import rostopic
import rosgraph
import numpy as np
from copy import deepcopy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from tf_quaternion.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties


vehicle_pub = dict()

odom_sub = dict()

get_world_props = None

get_model_props = None

marker = np.array([[0, 0.75], [-0.5, -0.25], [0.5, -0.25]])

def rot(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha)],
                     [np.sin(alpha), np.cos(alpha)]])

def odometry_callback(msg, name):
    global vehicle_pub
    if name not in vehicle_pub:
        return
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    orientation = np.array([msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w])

    yaw = euler_from_quaternion(orientation)[2]

    new_marker = deepcopy(marker)
    points = list()
    for i in range(new_marker.shape[0]):
        new_marker[i, :] = np.dot(rot(yaw-np.pi/2), new_marker[i, :])
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

    vehicle_pub[name].publish(new_poly)

def get_topics(name):
    return [t for t, _ in rosgraph.Master('/{}/'.format(name)).getPublishedTopics('') if not t.startswith('/rosout') and name in t]

def sub_odometry_topic(name):
    odom_topic_sub = None
    for t in get_topics(name):
        msg_class, _, _ = rostopic.get_topic_class(t)
        if msg_class == Odometry:
            odom_topic_sub = rospy.Subscriber(
                t, Odometry, lambda msg: odometry_callback(msg, name))
    return odom_topic_sub

def update_vehicle_list(event):
    """Call list of models in the Gazebo simulation and filter out the
    marine crafts.
    """
    global get_world_props
    if get_world_props is None:
        try:
            # Handle for world properties update function
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=2)
            get_world_props = rospy.ServiceProxy('/gazebo/get_world_properties',
                                                 GetWorldProperties)
        except rospy.ROSException:
            print('/gazebo/get_world_properties service is unavailable')

    global get_model_props
    if get_model_props is None:
        try:
            # Handle for retrieving model properties
            rospy.wait_for_service('/gazebo/get_model_properties')
            get_model_props = rospy.ServiceProxy('/gazebo/get_model_properties',
                                                 GetModelProperties)
        except rospy.ROSException:
            print('/gazebo/get_model_properties service is unavailable')
    try:
        global vehicle_pub
        global odom_sub
        msg = get_world_props()
        for model in msg.model_names:
            model_properties = get_model_props(model)
            if not model_properties.is_static and \
               rospy.has_param('/{}/robot_description'.format(model)) and \
               model not in vehicle_pub:
               odom_sub[model] = sub_odometry_topic(model)
               vehicle_pub[model] = rospy.Publisher('/{}/footprint'.format(model),
                                                    PolygonStamped,
                                                    queue_size=1)
    except rospy.ServiceException as e:
        print('Service call failed: {}'.format(e))

def main():
    update_timer = rospy.Timer(rospy.Duration(10), update_vehicle_list)

if __name__ == '__main__':
    print('Start publishing vehicle footprints to RViz')
    rospy.init_node('publish_footprints')

    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
