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
import numpy as np
import os
import yaml
from waypoint import Waypoint
from uuv_control_msgs.msg import WaypointSet as WaypointSetMessage
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class WaypointSet(object):

    FINAL_WAYPOINT_COLOR = [1.0, 131.0 / 255, 0.0]
    OK_WAYPOINT = [31. / 255, 106. / 255, 226. / 255]
    FAILED_WAYPOINT = [1.0, 0.0, 0.0]

    def __init__(self, scale=0.1):
        self._waypoints = list()
        self._violates_constraint = False
        self._scale = scale

    def __str__(self):
        if self.num_waypoints:
            msg = '================================\n'
            msg += 'List of waypoints\n'
            msg += '================================\n'
            for i in range(self.num_waypoints):
                msg += self.get_waypoint(i).__str__()                
                msg += '---\n'
            msg += 'Number of waypoints = %d\n' % self.num_waypoints
            msg += 'Number of valid waypoints = %d\n' % self.num_waypoints
            return msg
        else:
            return 'Waypoint set is empty'

    @property
    def num_waypoints(self):
        return len(self._waypoints)

    @property
    def x(self):
        return [wp.x for wp in self._waypoints]

    @property
    def y(self):
        return [wp.y for wp in self._waypoints]

    @property
    def z(self):
        return [wp.z for wp in self._waypoints]

    def clear_waypoints(self):
        self._waypoints = list()

    def set_constraint_status(self, index, flag):
        if index < 0 or index >= len(self._waypoints):
            return False
        self._waypoints[index].violates_constraint = flag
        return True

    def get_waypoint(self, index):
        if index < 0 or index >= len(self._waypoints):
            return None
        return self._waypoints[index]

    def add_waypoint(self, waypoint, add_to_beginning=False):
        if len(self._waypoints):
            # TODO: Test the current reference frame convention being used (default: Gazebo's ENU)
            if waypoint.z > 0:
                print 'Waypoint is above the sea surface, z=', waypoint.z
                return False
            if self._waypoints[-1] != waypoint:
                if not add_to_beginning:
                    self._waypoints.append(waypoint)
                else:
                    self._waypoints = [waypoint] + self._waypoints
            else:
                print 'Cannot add repeated waypoint'
                return False
        else:
            if not add_to_beginning:
                self._waypoints.append(waypoint)
            else:
                self._waypoints = [waypoint] + self._waypoints
        return True

    def add_waypoint_from_msg(self, msg):
        waypoint = Waypoint()
        waypoint.from_message(msg)
        return self.add_waypoint(waypoint)

    def get_start_waypoint(self):
        if len(self._waypoints):
            return self._waypoints[0]
        else:
            return None

    def get_last_waypoint(self):
        if len(self._waypoints):
            return self._waypoints[-1]
        return None

    def remove_waypoint(self, waypoint):
        new_waypoints = list()
        for point in self._waypoints:
            if point == waypoint:
                continue
            new_waypoints.append(point)
        self._waypoints = new_waypoints

    def read_from_file(self, filename):
        if not os.path.isfile(filename):
            print 'Invalid waypoint filename, file', filename
            return False
        try:
            self.clear_waypoints()
            with open(filename, 'r') as wp_file:
                wps = yaml.load(wp_file)
                for wp_data in wps:
                    wp = Waypoint(
                        x=wp_data['point'][0],
                        y=wp_data['point'][1],
                        z=wp_data['point'][2],
                        max_forward_speed=wp_data['max_forward_speed'],
                        heading_offset=wp_data['heading'],
                        use_fixed_heading=wp_data['use_fixed_heading'])
                    self.add_waypoint(wp)
        except:
            print 'Error while loading the file'
            return False
        return True

    def export_to_file(self, path, filename):
        try:
            wp_data = list()
            for wp in self._waypoints:
                wp_elem = dict(point=[float(wp.x), float(wp.y), float(wp.z)],
                               max_forward_speed=float(wp._max_forward_speed),
                               heading=float(wp._heading_offset if wp._heading_offset is not None else 0.0),
                               use_fixed_heading=bool(wp._use_fixed_heading))
                wp_data.append(wp_elem)
            with open(os.path.join(path, filename), 'w') as wp_file:
                yaml.dump(wp_data, wp_file, default_flow_style=False)
            return True
        except:
            print 'Error occured while exporting waypoint file'
            return False

    def to_message(self):
        msg = WaypointSetMessage()
        msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = 'world'
        msg.waypoints = list()
        for wp in self._waypoints:
            msg.waypoints.append(wp.to_message())
        return msg

    def from_message(self, msg):
        self.clear_waypoints()
        for pnt in msg.waypoints:
            self.add_waypoint_from_msg(pnt)

    def dist_to_waypoint(self, pos, index=0):
        wp = self.get_waypoint(index)
        if wp is not None:
            return wp.dist(pos)
        return None

    def to_path_marker(self, clear=False):
        path = Path()
        t = rospy.Time.now()
        path.header.stamp = t
        path.header.frame_id = 'world'
        if self.num_waypoints > 1 and not clear:
            for i in range(self.num_waypoints):
                wp = self.get_waypoint(i)
                pose = PoseStamped()
                pose.header.stamp = rospy.Time(i)
                pose.header.frame_id = 'world'
                pose.pose.position.x = wp.x
                pose.pose.position.y = wp.y
                pose.pose.position.z = wp.z
                path.poses.append(pose)
        return path

    def to_marker_list(self, clear=False):
        list_waypoints = MarkerArray()
        t = rospy.Time.now()
        if self.num_waypoints == 0 or clear:
            marker = Marker()
            marker.header.stamp = t
            marker.header.frame_id = 'world'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = 3
            list_waypoints.markers.append(marker)
        else:
            for i in range(self.num_waypoints):
                wp = self.get_waypoint(i)
                marker = Marker()
                marker.header.stamp = t
                marker.header.frame_id = 'world'
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = wp.x
                marker.pose.position.y = wp.y
                marker.pose.position.z = wp.z
                marker.scale.x = self._scale
                marker.scale.y = self._scale
                marker.scale.z = self._scale
                marker.color.a = 1.0
                if wp == self.get_last_waypoint():
                    color = wp.get_final_color()
                else:
                    color = wp.get_color()
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                list_waypoints.markers.append(marker)
        return list_waypoints

    def generate_circle(self, radius, center, num_points, max_forward_speed,
                        theta_offset=0.0, heading_offset=0.0, append=False):
        if radius <= 0:
            print 'Invalid radius, value=', radius
            return False

        if num_points <= 0:
            print 'Invalid number of samples, value=', num_points
            return False

        if max_forward_speed <= 0:
            print 'Invalid absolute maximum velocity, value=', max_forward_speed
            return False

        if not append:
            # Clear current list
            self.clear_waypoints()

        step_theta = 2 * np.pi / num_points
        for i in range(num_points):
            angle = i * step_theta + theta_offset
            x = np.cos(angle) * radius + center.x
            y = np.sin(angle) * radius + center.y
            z = center.z
            wp = Waypoint(x, y, z, max_forward_speed,
                          heading_offset)
            self.add_waypoint(wp)
        return True

    def generate_helix(self, radius, center, num_points, max_forward_speed, delta_z,
                       num_turns, theta_offset=0.0, heading_offset=0.0,
                       append=False):
        if radius <= 0:
            print 'Invalid radius, value=', radius
            return False

        if num_points <= 0:
            print 'Invalid number of samples, value=', num_points
            return False

        if num_turns <= 0:
            print 'Invalid number of turns, value=', num_points
            return False

        if max_forward_speed <= 0:
            print 'Invalid absolute maximum velocity, value=', max_forward_speed
            return False

        if not append:
            # Clear current list
            self.clear_waypoints()

        total_angle = 2 * np.pi * num_turns
        step_angle = total_angle / num_points
        step_z = float(delta_z) / num_points
        for i in range(num_points):
            angle = theta_offset + i * step_angle
            x = radius * np.cos(angle) + center.x
            y = radius * np.sin(angle) + center.y
            z = step_z * i + center.z

            wp = Waypoint(x, y, z, max_forward_speed,
                          heading_offset)
            self.add_waypoint(wp)
        return True
