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
from copy import deepcopy
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion
import uuv_control_msgs.msg as uuv_control_msgs
from nav_msgs.msg import Path
from .wp_trajectory_generator import WPTrajectoryGenerator
from .waypoint_set import WaypointSet
from .trajectory_point import TrajectoryPoint


class TrajectoryGenerator(object):
    def __init__(self, full_dof=False, stamped_pose_only=False):
        self._points = None
        self._time = None
        self._this_pnt = None
        self._is_full_dof = full_dof
        self._stamped_pose_only = stamped_pose_only
        self._wp_interp_on = False
        self._wp_interp = WPTrajectoryGenerator(full_dof=full_dof, stamped_pose_only=stamped_pose_only)

        self._has_started = False
        self._is_finished = False

    @property
    def points(self):
        if self._points is not None:
            return self._points
        elif self._wp_interp_on:
            return self._wp_interp.get_samples(0.005)
        else:
            return None

    def use_finite_diff(self, flag):
        self._wp_interp.use_finite_diff = flag

    def is_using_finite_diff(self):
        return self._wp_interp.use_finite_diff

    def set_stamped_pose_only(self, flag):
        self._wp_interp.stamped_pose_only = flag

    def is_using_stamped_pose_only(self):
        return self._wp_interp.stamped_pose_only 

    def set_interp_method(self, method):
        return self._wp_interp.set_interpolation_method(method)

    def get_interp_method(self):
        return self._wp_interp.get_interpolation_method()

    def _reset(self):
        self._points = None
        self._time = None
        self._this_pnt = None
        self._has_started = False
        self._is_finished = False

    def get_trajectory_as_message(self):
        """
        Return the trajectory points as a Trajectory type message. If waypoints
        are currently in use, then sample the interpolated path and return the
        poses only.
        """
        
        try:
            if self.points is None:
                return None
            msg = uuv_control_msgs.Trajectory()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'world'
            for pnt in self.points:
                # FIXME Sometimes the time stamp of the point is NaN
                msg.points.append(pnt.to_message())
            return msg
        except:
            print 'Error during creation of trajectory message'
            return None

    def is_using_waypoints(self):
        """Return true if the waypoint interpolation is being used."""
        return self._wp_interp_on

    def set_waypoints(self, waypoints):
        """Initializes the waypoint interpolator with a set of waypoints."""
        if self._wp_interp.init_waypoints(waypoints):
            self._wp_interp_on = True
            return True
        else:
            return False

    def get_waypoints(self):
        """
        Return the waypoints used by the waypoint interpolator,
        if any exist.
        """

        if not self.is_using_waypoints():
            return None
        return self._wp_interp.get_waypoints()

    def add_waypoint(self, waypoint, add_to_beginning=False):
        """
        Add waypoint to the current waypoint set, if one has been initialized.
        """
        if not self._wp_interp_on:
            return False
        self._wp_interp.add_waypoint(waypoint, add_to_beginning)
        return True

    def add_trajectory_point(self, pnt):
        """
        If a trajectory set is currently being used in the interpolation
        process, add a trajectory point to the set.
        """

        if not self._wp_interp_on:
            if self._points is None:
                self._points = list()
                self._time = list()
            if len(self._points) > 1:
                if pnt.t <= self._points[-1].t:
                    return False
            self._points.append(pnt)
            self._time.append(pnt.t)
            return True
        else:
            print 'Cannot add trajectory point! Generator is in waypoint interpolation mode!'
            return False

    def add_trajectory_point_from_msg(self, msg):
        if not self._wp_interp_on:
            pnt = TrajectoryPoint()
            if pnt.from_message(msg):
                self.add_trajectory_point(pnt)
                return True
            else:
                print 'Error converting message to trajectory point'
                return False
        else:
            print 'Cannot add trajectory point! Generator is in waypoint interpolation mode!'
            return False

    def set_duration(self, t):
        if not self._wp_interp_on:
            print 'Waypoint interpolation is not activated'
            return False
        else:
            return self._wp_interp.set_duration(t)

    def get_max_time(self):
        if self._points is None and not self._wp_interp_on:
            return None
        if self._wp_interp_on:
            return self._wp_interp.get_max_time()
        else:
            return self._points[-1].t

    def is_running(self):
        return self._has_started and not self._is_finished

    def has_started(self):
        if self._wp_interp_on:
            return self._wp_interp.started
        else:
            return self._has_started

    def has_finished(self):
        if self._wp_interp_on:
            return self._wp_interp.is_finished()
        else:
            return self._is_finished

    def init_from_trajectory_message(self, msg):
        self._wp_interp_on = False
        self._points = None
        self._time = None

        last_t = None
        for p_msg in msg.points:
            t = p_msg.header.stamp.to_sec()

            if last_t is None:
                last_t = t
            else:
                if t <= last_t:
                    print 'Trajectory should be given a growing value of time'
                    self._reset()
                    return False
            self.add_trajectory_point_from_msg(p_msg)
        self._has_started = True
        return True

    def init_from_waypoint_message(self, msg):
        wp_set = WaypointSet.from_message(msg)
        self._wp_interp.init_waypoints(wp_set)
        self._wp_interp_on = True

    def init_from_waypoint_file(self, filename):
        wp_set = WaypointSet()
        success = wp_set.read_from_file(filename)
        if success:
            self._wp_interp.init_waypoints(wp_set)
            self._wp_interp_on = True
        return success

    def gen_trajectory_message(self):
        if self._points is None:
            return None
        msg = uuv_control_msgs.Trajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        if not self.is_using_waypoints:
            for p in self._points:
                p_msg = uuv_control_msgs.TrajectoryPoint()
                p_msg.header.stamp = rospy.Time(p.t)
                p_msg.pose.position = Vector3(*p.p)
                p_msg.pose.orientation = Quaternion(*p.q)
                p_msg.velocity.linear = Vector3(*p.v)
                p_msg.velocity.angular = Vector3(*p.w)
                p_msg.acceleration.linear = Vector3(*p.a)
                p_msg.acceleration.angular = Vector3(*p.alpha)
                msg.point.append(p_msg)
        else:
            dt = 0.05 * self._wp_interp.get_max_time()
            for ti in np.arange(0, self._wp_interp.get_max_time(), dt):
                pnt = self._wp_interp.interpolate(ti)
                p_msg = uuv_control_msgs.TrajectoryPoint()
                p_msg.header.stamp = rospy.Time(pnt.t)
                p_msg.pose.position = Vector3(*pnt.p)
                p_msg.pose.orientation = Quaternion(*pnt.q)
                p_msg.velocity.linear = Vector3(*pnt.v)
                p_msg.velocity.angular = Vector3(*pnt.w)
                p_msg.acceleration.linear = Vector3(*pnt.a)
                p_msg.acceleration.angular = Vector3(*pnt.alpha)
                msg.point.append(p_msg)
        return msg

    def get_path_message(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'world'
        path_msg.poses = list()

        for point in self._local_planner.points:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time(point.t)
            pose.pose.position = Vector3(*point.p)
            pose.pose.orientation = Quaternion(*point.q)
            path_msg.poses.append(pose)

    def set_start_time(self, t):
        if self._wp_interp_on:
            self._wp_interp.set_start_time(t)
            return True
        return False

    def interpolate(self, t):
        if not self._wp_interp_on:
            self._this_pnt = TrajectoryPoint()
            if self._points is None:
                return None
            if len(self._points) == 0:
                return None

            if type(self._time) == list:
                self._time = np.array(self._time)

            # Interpolate the given trajectory
            self._this_pnt.t = t
            if t <= self._points[0].t:
                self._this_pnt.pos = deepcopy(self._points[0].pos)
                self._this_pnt.rotq = deepcopy(self._points[0].rotq)
                self._has_started = False
                self._has_finished = False
            if t >= self._points[-1].t:
                self._this_pnt.pos = deepcopy(self._points[-1].pos)
                self._this_pnt.rotq = deepcopy(self._points[-1].rotq)
                self._has_started
                self._is_finished = True
            else:
                self._has_started = True
                self._has_finished = False
                idx = np.argmin(np.abs(self._time - t))
                if idx == 0:
                    self._this_pnt = deepcopy(self._points[0])
                else:
                    if t < self._points[idx].t:
                        p_this = self._points[idx]
                        p_last = self._points[idx - 1]
                    else:
                        p_this = self._points[idx + 1]
                        p_last = self._points[idx]
                    dt = p_this.t - p_last.t
                    w1 = (t - p_last.t) / dt
                    w0 = (p_this.t - t) / dt
                    self._this_pnt.pos = w0*p_last.pos + w1*p_this.pos
                    self._this_pnt.rotq = w0*p_last.rotq + w1*p_this.rotq
                    self._this_pnt.vel = w0*p_last.vel + w1*p_this.vel
                    self._this_pnt.acc = w0*p_last.acc + w1*p_this.acc
        else:
            self._this_pnt = self._wp_interp.interpolate(t)

        return self._this_pnt
