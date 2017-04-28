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
import os
import yaml
from datetime import datetime
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint, WaypointSet
import uuv_trajectory_generator


class TrajectoryMarkerPublisher:
    def __init__(self):
        self._trajectory_sub = rospy.Subscriber(
            'trajectory', Trajectory, self._update_trajectory)

        self._waypoints_sub = rospy.Subscriber(
            'waypoints', WaypointSet, self._update_waypoints)

        # Vehicle state flags
        self._is_auto_on = False
        self._is_traj_tracking_on = False
        self._is_station_keeping_on = False

        self._automatic_mode_sub = rospy.Subscriber(
            'automatic_on', Bool, self._update_auto_mode)

        self._traj_tracking_mode_sub = rospy.Subscriber(
            'trajectory_tracking_on', Bool, self._update_traj_tracking_mode)

        self._station_keeping_mode_sub = rospy.Subscriber(
            'station_keeping_on', Bool, self._update_station_keeping_mode)

        # Waypoint set received
        self._waypoints = None
        # Trajectory received
        self._trajectory = None

        self._output_dir = None
        if rospy.has_param('~output_dir'):
            self._output_dir = rospy.get_param('~output_dir')
            if not os.path.isdir(self._output_dir):
                print 'Invalid output directory, not saving the files, dir=', self._output_dir
                self._output_dir = None
            else:
                self._output_dir = os.path.join(self._output_dir, rospy.get_namespace().replace('/', ''))
                if not os.path.isdir(self._output_dir):
                    os.makedirs(self._output_dir)

        # Visual marker publishers
        self._trajectory_path_pub = rospy.Publisher(
            'trajectory_marker', Path, queue_size=1)

        self._waypoint_markers_pub = rospy.Publisher(
            'waypoint_markers', MarkerArray, queue_size=1)

        self._waypoint_path_pub = rospy.Publisher(
            'waypoint_path_marker', Path, queue_size=1)

        self._update_markers_timer = rospy.Timer(
            rospy.Duration(0.5), self._update_markers)

    def _update_markers(self, event):
        if self._waypoints is None:
            waypoint_path_marker = Path()
            t = rospy.Time.now()
            waypoint_path_marker.header.stamp = t
            waypoint_path_marker.header.frame_id = 'world'

            waypoint_marker = MarkerArray()
            marker = Marker()
            marker.header.stamp = t
            marker.header.frame_id = 'world'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = 3
        else:
            waypoint_path_marker = self._waypoints.to_path_marker()
            waypoint_marker = self._waypoints.to_marker_list()

        self._waypoint_path_pub.publish(waypoint_path_marker)
        self._waypoint_markers_pub.publish(waypoint_marker)

        traj_marker = Path()
        traj_marker.header.stamp = rospy.Time.now()
        traj_marker.header.frame_id = 'world'

        if self._trajectory is not None:
            for pnt in self._trajectory.points:
                p_msg = PoseStamped()
                p_msg.header.stamp = pnt.header.stamp
                p_msg.pose = pnt.pose
                traj_marker.poses.append(p_msg)

        self._trajectory_path_pub.publish(traj_marker)
        return True

    def _update_trajectory(self, msg):
        self._trajectory = msg

    def _update_waypoints(self, msg):
        self._waypoints = uuv_trajectory_generator.WaypointSet()
        self._waypoints.from_message(msg)

    def _update_auto_mode(self, msg):
        self._is_auto_on = msg.data

    def _update_station_keeping_mode(self, msg):
        self._is_station_keeping_on = msg.data

    def _update_traj_tracking_mode(self, msg):
        self._is_traj_tracking_on = msg.data

if __name__ == '__main__':
    print('Starting trajectory and waypoint marker publisher')
    rospy.init_node('trajectory_marker_publisher')

    try:
        node = TrajectoryMarkerPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
