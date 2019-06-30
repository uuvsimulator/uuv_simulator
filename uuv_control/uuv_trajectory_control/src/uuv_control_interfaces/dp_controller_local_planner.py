# Copyright (c) 2016-2019 The UUV Simulator Authors.
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
import logging
import sys
import time
import numpy as np
from copy import deepcopy
from os.path import isfile
from threading import Lock, Event

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist
from uuv_control_msgs.srv import *
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint, WaypointSet
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

import uuv_trajectory_generator
import uuv_waypoints
from tf_quaternion.transformations import quaternion_about_axis, quaternion_multiply, \
    quaternion_inverse, quaternion_matrix, euler_from_quaternion, quaternion_from_euler

from ._log import get_logger


class DPControllerLocalPlanner(object):
    """Local planner for the dynamic positioning controllers 
    to interpolate trajectories and generate trajectories from 
    interpolated waypoint paths.

    > *Input parameters*

    * `full_dof` (*type:* `bool`, *default:* `False`): If `True`, 
    the reference trajectory reference will be computed for 6 DoF,
    otherwise, 4 DoF `(x, y, z, yaw)`.
    * `stamped_pose_only` (*type:* `bool`, *default:* `False`): If 
    `True`, only stamped poses will be generated as a reference, with
    velocity and acceleration reference being set to zero.
    * `thrusters_only` (*type:* `bool`, *default:* `True`): If `False`,
    the idle mode will be used to keep the vehicle moving. 

    > *ROS parameters*

    * `max_forward_speed` (*type:* `float`, *default:* `1.0`): Maximum
    allowed forward speed.
    * `idle_radius` (*type:* `float`, *default:* `10.0`): Radius of the circle 
    path generated when an AUV is in idle mode.
    * `inertial_frame_id` (*type:* `str`): Name of the inertial frame used, 
    options are `world` or `world_ned`.
    * `timeout_idle_mode` (*type:* `float`): Timeout at the start or after
    a trajectory is finished where the AUV is set to start idle mode path.
    * `look_ahead_delay` (*type:* `float`): Look ahead delay in seconds. This
    parameters will offset the interpolation of the trajectory in the given
    amount of seconds to compute the look-ahead target for AUVs.

    !!! warning

        The parameters for the path interpolators must also be provided when 
        starting a node that includes the local planner, since the interpolators 
        are initialized by the local planner.

    > *ROS publishers*

    * `trajectory` (*type:* `uuv_control_msgs.Trajectory`): Generated trajectory or
    stamped pose path.
    * `waypoints` (*type:* `uuv_control_msgs.WaypointSet`): Set of waypoints provided
    as input for the interpolator
    * `station_keeping_on` (*type:* `std_msgs.Bool`): Status of the station keeping mode
    * `automatic_on` (*type:* `std_msgs.Bool`): Status of automatic model. If `False`
    the vehicle can receive control inputs from a teleop node.
    * `trajectory_tracking_on` (*type:* `std_msgs.Bool`): Sets the output flag to `True`
    when trajectory tracking is ongoing
    * `interpolator_visual_markers` (*type:* `visualization_msgs.MarkerArray`): Helper
    visual markers from the interpolator class.
    * `time_to_target` (*type:* `std_msgs.Float64`): Estimated time to target in seconds.

    > *ROS services*

    * `hold_vehicle` (*type:* `uuv_control_msgs.Hold`)
    * `start_waypoint_list` (*type:* `uuv_control_msgs.InitWaypointSet`)
    * `start_circular_trajectory` (*type:* `uuv_control_msgs.InitCircularTrajectory`)
    * `start_helical_trajectory` (*type:* `uuv_control_msgs.InitHelicalTrajectory`)
    * `init_waypoints_from_file` (*type:* `uuv_control_msgs.InitWaypointsFromFile`)
    * `go_to` (*type:* `uuv_control_msgs.GoTo`)
    * `go_to_incremental` (*type:* `uuv_control_msgs.GoToIncremental`)
    """

    def __init__(self, full_dof=False, stamped_pose_only=False, thrusters_only=True):
        self._logger = get_logger()

        self._lock = Lock()

        self._traj_interpolator = uuv_trajectory_generator.TrajectoryGenerator(
            full_dof=full_dof, stamped_pose_only=stamped_pose_only)

        # Max. allowed forward speed
        self._max_forward_speed = rospy.get_param('~max_forward_speed', 1.0)

        self._idle_circle_center = None
        self._idle_z = None

        self._logger.info('Max. forward speed [m/s]=%.2f' % self._max_forward_speed)

        self._idle_radius = rospy.get_param('~idle_radius', 10.0)
        assert self._idle_radius > 0

        self._logger.info('Idle circle radius [m] = %.2f' % self._idle_radius)

        # Is underactuated?
        self._is_underactuated = rospy.get_param('~is_underactuated', False)

        self.inertial_frame_id = 'world'
        self.transform_ned_to_enu = None
        self.q_ned_to_enu = None
        if rospy.has_param('~inertial_frame_id'):
            self.inertial_frame_id = rospy.get_param('~inertial_frame_id')
            assert len(self.inertial_frame_id) > 0
            assert self.inertial_frame_id in ['world', 'world_ned']

        self._logger.info('Inertial frame ID=' + self.inertial_frame_id)

        rospy.set_param('inertial_frame_id', self.inertial_frame_id)

        try:
            import tf2_ros

            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)

            tf_trans_ned_to_enu = tf_buffer.lookup_transform(
                'world', 'world_ned', rospy.Time(),
                rospy.Duration(10))
            
            self.q_ned_to_enu = np.array(
                [tf_trans_ned_to_enu.transform.rotation.x,
                tf_trans_ned_to_enu.transform.rotation.y,
                tf_trans_ned_to_enu.transform.rotation.z,
                tf_trans_ned_to_enu.transform.rotation.w])
        except Exception as ex:
            self._logger.warning(
                'Error while requesting ENU to NED transform'
                ', message={}'.format(ex))
            self.q_ned_to_enu = quaternion_from_euler(2 * np.pi, 0, np.pi)
                                
        self.transform_ned_to_enu = quaternion_matrix(
                self.q_ned_to_enu)[0:3, 0:3]

        if self.transform_ned_to_enu is not None:
            self._logger.info('Transform world_ned (NED) to world (ENU)=\n' +
                                str(self.transform_ned_to_enu))

        self._logger.info('Inertial frame ID=' + self.inertial_frame_id)
        self._logger.info('Max. forward speed = ' +
                          str(self._max_forward_speed))

        for method in self._traj_interpolator.get_interpolator_tags():
            if rospy.has_param('~' + method):
                self._logger.info('Parameters for interpolation method <%s> found' % method)
                params = rospy.get_param('~' + method)
                self._logger.info('\t' + str(params))

                self._traj_interpolator.set_interpolator_parameters(method, params)
            else:
                self._logger.info('No parameters for interpolation method <%s> found' % method)

        # dt used to compute the pose reference from the joystick input
        self._dt = 0.0
        # Time stamp for the last velocity reference received
        self._last_teleop_update = None
        # Flag to indicate if the teleoperation node is active
        self._is_teleop_active = False
        # Teleop node twist message
        self._teleop_vel_ref = None

        self.init_odom_event = Event()
        self.init_odom_event.clear()

        self._timeout_idle_mode = rospy.get_param('~timeout_idle_mode', 5)
        self._start_count_idle = rospy.get_time()

        self._thrusters_only = thrusters_only

        if not self._thrusters_only:
            self._look_ahead_delay = rospy.get_param('~look_ahead_delay', 3.0)
        else:
            self._look_ahead_delay = 0.0

        self._station_keeping_center = None

        # Publishing topic for the trajectory given to the controller
        self._trajectory_pub = rospy.Publisher('trajectory',
                                               Trajectory,
                                               queue_size=1)
        # Publishing waypoints
        self._waypoints_pub = rospy.Publisher('waypoints',
                                               WaypointSet,
                                               queue_size=1)

        self._station_keeping_pub = rospy.Publisher('station_keeping_on',
                                                    Bool,
                                                    queue_size=1)

        self._automatic_control_pub = rospy.Publisher('automatic_on',
                                                      Bool,
                                                      queue_size=1)

        self._traj_tracking_pub = rospy.Publisher('trajectory_tracking_on',
                                                  Bool,
                                                  queue_size=1)

        self._interp_visual_markers = rospy.Publisher('interpolator_visual_markers',
                                                      MarkerArray,
                                                      queue_size=1)

        self._teleop_sub = rospy.Subscriber('cmd_vel', Twist, self._update_teleop)

        self._waypoints_msg = None
        self._trajectory_msg = None

        # Subscribing topic for the trajectory given to the controller
        self._input_trajectory_sub = rospy.Subscriber(
            'input_trajectory', Trajectory, self._update_trajectory_from_msg)

        self._max_time_pub = rospy.Publisher('time_to_target', Float64, queue_size=1)

        self._traj_info_update_timer = rospy.Timer(rospy.Duration(0.2),
            self._publish_trajectory_info)
        # Flag to activate station keeping
        self._station_keeping_on = True
        # Flag to set vehicle control to automatic
        self._is_automatic = True
        # Flag true if a trajectory is being tracked
        self._traj_running = False
        # Current vehicle pose
        self._vehicle_pose = None
        # Current reference point
        self._this_ref_pnt = None
        # Flag that indicates that a waypoint set has been initialized
        self._smooth_approach_on = False
        # Time stamp for received trajectory
        self._stamp_trajectory_received = 0.0
        # Dictionary of services
        self._services = dict()
        self._services['hold_vehicle'] = rospy.Service(
            'hold_vehicle', Hold, self.hold_vehicle)
        self._services['start_waypoint_list'] = rospy.Service(
            'start_waypoint_list', InitWaypointSet, self.start_waypoint_list)
        self._services['start_circular_trajectory'] = rospy.Service(
            'start_circular_trajectory', InitCircularTrajectory,
            self.start_circle)
        self._services['start_helical_trajectory'] = rospy.Service(
            'start_helical_trajectory', InitHelicalTrajectory,
            self.start_helix)
        self._services['init_waypoints_from_file'] = rospy.Service(
            'init_waypoints_from_file', InitWaypointsFromFile,
            self.init_waypoints_from_file)
        self._services['go_to'] = rospy.Service('go_to', GoTo, self.go_to)
        self._services['go_to_incremental'] = rospy.Service(
            'go_to_incremental', GoToIncremental, self.go_to_incremental)

    def __del__(self):
        """Remove logging message handlers"""
        while self._logger.handlers:
            self._logger.handlers.pop()

    def _transform_position(self, vec, target, source):
        """Transform the position vector between `world` and `world_ned`.
        
        > *Input arguments*
        
        * `vec` (*type:* `numpy.array`): Position vector
        * `target` (*type:* `str`): Target frame
        * `source` (*type:* `str`): Source frame
        
        > *Returns*
        
        `numpy.array`: Transformed vector
        """
        if target == source:
            return vec
        if target == 'world':
            return np.dot(self.transform_ned_to_enu, vec)
        if target == 'world_ned':
            return np.dot(self.transform_ned_to_enu.T, vec)

    def _transform_waypoint(self, waypoint):
        """Transform position vector of a waypoint between 
        `world` and `world_ned` frames.
        
        > *Input arguments*
        
        * `waypoint` (*type:* `uuv_waypoints.Waypoint`): Input waypoint
        
        > *Returns*
        
        `uuv_waypoints.Waypoint`: Transformed waypoint
        """
        output = deepcopy(waypoint)
        output.pos = self._transform_position(output.pos,
                                              self.inertial_frame_id,
                                              output.inertial_frame_id)
        output.inertial_frame_id = self.inertial_frame_id
        output.max_forward_speed = min(waypoint.max_forward_speed, self._max_forward_speed)
        return output

    def _transform_waypoint_set(self, waypoint_set):
        """Apply transformation between `world` and 'world_ned` frames
        to waypoints in a waypoint set.
        
        > *Input arguments*
        
        * `waypoint_set` (*type:* `uuv_waypoins.WaypointSet`): Set of waypoints
        
        > *Returns*
        
        `uuv_waypoins.WaypointSet`: Set of transformed waypoints
        """
        output = uuv_waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)
        for i in range(waypoint_set.num_waypoints):
            wp = self._transform_waypoint(waypoint_set.get_waypoint(i))
            output.add_waypoint(wp)
        return output

    def _apply_workspace_constraints(self, waypoint_set):
        """Filter out waypoints that are positioned above
        sea surface, namely `z > 0` if the inertial frame is
        `world`, or `z < 0` if the inertial frame is `world_ned`.
        
        > *Input arguments*
        
        * `waypoint_set` (*type:* `uuv_waypoins.WaypointSet`): Set of waypoints
        
        > *Returns*
        
        `uuv_waypoins.WaypointSet`: Filtered set of waypoints
        """
        wp_set = uuv_waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)
        for i in range(waypoint_set.num_waypoints):
            wp = waypoint_set.get_waypoint(i)
            if wp.z > 0 and self.inertial_frame_id == 'world':
                continue
            if wp.z < 0 and self.inertial_frame_id == 'world_ned':
                continue
            wp_set.add_waypoint(wp)
        return wp_set

    def _publish_trajectory_info(self, event):
        """Publish messages for the waypoints, trajectory and 
        debug flags.
        """
        if self._waypoints_msg is not None:
            self._waypoints_pub.publish(self._waypoints_msg)
        if self._trajectory_msg is not None:
            self._trajectory_pub.publish(self._trajectory_msg)
        markers = self._traj_interpolator.get_visual_markers()
        if markers is not None:
            self._interp_visual_markers.publish(markers)
        else:
            self._interp_visual_markers.publish(MarkerArray())
        self._station_keeping_pub.publish(Bool(self._station_keeping_on))
        self._automatic_control_pub.publish(Bool(self._is_automatic))
        self._traj_tracking_pub.publish(Bool(self._traj_running))
        return True

    def _update_trajectory_info(self):
        """Update the trajectory message."""
        self._waypoints_msg = WaypointSet()
        if self._traj_interpolator.is_using_waypoints():
            wps = self._traj_interpolator.get_waypoints()
            if wps is not None:
                wps.inertial_frame_id = self.inertial_frame_id
                self._waypoints_msg = wps.to_message()
                self._waypoints_msg.header.frame_id = self.inertial_frame_id
        msg = self._traj_interpolator.get_trajectory_as_message()
        if msg is not None:
            msg.header.frame_id = self.inertial_frame_id
            self._trajectory_msg = msg
            self._logger.info('Updating the trajectory information')
        else:
            self._trajectory_msg = None
            self._logger.error('Error generating trajectory message')

    def _update_teleop(self, msg):
        """Callback to the twist teleop subscriber."""
        # Test whether the vehicle is in automatic mode (following a given
        # trajectory)
        if self._is_automatic:
            self._teleop_vel_ref = None
            return

        # If this is the first twist message since the last time automatic mode
        # was turned off, then just update the teleop timestamp and wait for
        # the next message to allow computing pose and velocity reference.
        if self._last_teleop_update is None:
            self._teleop_vel_ref = None
            self._last_teleop_update = rospy.get_time()
            return

        # Store twist reference message
        self._teleop_vel_ref = msg

        # Set the teleop mode is active only if any of the linear velocity components and
        # yaw rate are non-zero
        vel = np.array([self._teleop_vel_ref.linear.x, self._teleop_vel_ref.linear.y, self._teleop_vel_ref.linear.z, self._teleop_vel_ref.angular.z])
        self._is_teleop_active = np.abs(vel).sum() > 0

        # Store time stamp
        self._last_teleop_update = rospy.get_time()

    def _calc_teleop_reference(self):
        """Compute pose and velocity reference using the 
        joystick linear and angular velocity input.
        """
        # Check if there is already a timestamp for the last received reference
        # message from the teleop node
        if self._last_teleop_update is None:
            self._is_teleop_active = False

        # Compute time step
        self._dt = rospy.get_time() - self._last_teleop_update

        # Compute the pose and velocity reference if the computed time step is
        # positive and the twist teleop message is valid
        if self._dt > 0 and self._teleop_vel_ref is not None and self._dt < 0.1:
            speed = np.sqrt(self._teleop_vel_ref.linear.x**2 + self._teleop_vel_ref.linear.y**2)
            vel = np.array([self._teleop_vel_ref.linear.x, self._teleop_vel_ref.linear.y, self._teleop_vel_ref.linear.z])
            # Cap the forward speed if needed
            if speed > self._max_forward_speed:
                vel[0] *= self._max_forward_speed / speed
                vel[1] *= self._max_forward_speed / speed

            vel = np.dot(self._vehicle_pose.rot_matrix, vel)

            # Compute pose step
            step = uuv_trajectory_generator.TrajectoryPoint()
            step.pos = np.dot(self._vehicle_pose.rot_matrix, vel * self._dt)
            step.rotq = quaternion_about_axis(self._teleop_vel_ref.angular.z * self._dt, [0, 0, 1])

            # Compute new reference
            ref_pnt = uuv_trajectory_generator.TrajectoryPoint()
            ref_pnt.pos = self._vehicle_pose.pos + step.pos

            ref_pnt.rotq = quaternion_multiply(self.get_vehicle_rot(), step.rotq)

            # Cap the pose reference in Z to stay underwater
            if ref_pnt.z > 0:
                ref_pnt.z = 0.0
                ref_pnt.vel = [vel[0], vel[1], 0, 0, 0, self._teleop_vel_ref.angular.z]
            else:
                ref_pnt.vel = [vel[0], vel[1], vel[2], 0, 0, self._teleop_vel_ref.angular.z]

            ref_pnt.acc = np.zeros(6)
        else:
            self._is_teleop_active = False
            ref_pnt = deepcopy(self._vehicle_pose)
        return ref_pnt

    def _calc_smooth_approach(self):
        """Add the current vehicle position as waypoint 
        to allow a smooth approach to the given trajectory.
        """
        if self._vehicle_pose is None:
            self._logger.error('Simulation not properly initialized yet, ignoring approach...')
            return
        if not self._traj_interpolator.is_using_waypoints():
            self._logger.error('Not using the waypoint interpolation method')
            return

        heading = euler_from_quaternion(self.get_vehicle_rot())[2]

        if self._thrusters_only:
            init_wp = uuv_waypoints.Waypoint(
                x=self._vehicle_pose.pos[0],
                y=self._vehicle_pose.pos[1],
                z=self._vehicle_pose.pos[2],
                max_forward_speed=self._traj_interpolator.get_waypoints().get_waypoint(0).max_forward_speed,
                heading_offset=self._traj_interpolator.get_waypoints().get_waypoint(0).heading_offset)
        else:
            max_speed = self._traj_interpolator.get_waypoints().get_waypoint(0).max_forward_speed
            init_wp = uuv_waypoints.Waypoint(
                x=self._vehicle_pose.pos[0],# + max_speed / self._look_ahead_delay * np.cos(heading),
                y=self._vehicle_pose.pos[1],# + max_speed / self._look_ahead_delay * np.sin(heading),
                z=self._vehicle_pose.pos[2],
                max_forward_speed=max_speed,
                heading_offset=self._traj_interpolator.get_waypoints().get_waypoint(0).heading_offset)
        first_wp = self._traj_interpolator.get_waypoints().get_waypoint(0)

        dx = first_wp.x - init_wp.x
        dy = first_wp.y - init_wp.y
        dz = first_wp.z - init_wp.z

        # One new waypoint at each meter
        self._logger.info('Adding waypoints to approach the first position in the given waypoint set')
        steps = int(np.floor(first_wp.dist(init_wp.pos)) / 10)
        if steps > 0 and self._traj_interpolator.get_interp_method() != 'dubins':
            for i in range(1, steps):
                wp = uuv_waypoints.Waypoint(
                    x=first_wp.x - i * dx / steps,
                    y=first_wp.y - i * dy / steps,
                    z=first_wp.z - i * dz / steps,
                    max_forward_speed=self._traj_interpolator.get_waypoints().get_waypoint(0).max_forward_speed)
                self._traj_interpolator.add_waypoint(wp, add_to_beginning=True)
        self._traj_interpolator.add_waypoint(init_wp, add_to_beginning=True)
        self._update_trajectory_info()

    def is_station_keeping_on(self):
        """Return `True`, if vehicle is holding its position."""
        return self._station_keeping_on

    def is_automatic_on(self):
        """Return `True` if vehicle if following a trajectory in 
        automatic mode.
        """
        return self._is_automatic

    def set_station_keeping(self, is_on=True):
        """Set station keeping mode flag.
        
        > *Input arguments* 

        * `is_on` (*type:* `bool`, *default:* `True`): Station keeping flag
        """
        self._station_keeping_on = is_on
        self._logger.info('STATION KEEPING MODE = ' + ('ON' if is_on else 'OFF'))

    def set_automatic_mode(self, is_on=True):
        """Set automatic mode flag."""
        self._is_automatic = is_on
        self._logger.info('AUTOMATIC MODE = ' + ('ON' if is_on else 'OFF'))

    def set_trajectory_running(self, is_on=True):
        """Set trajectory tracking flag."""
        self._traj_running = is_on
        self._logger.info('TRAJECTORY TRACKING = ' + ('ON' if is_on else 'OFF'))

    def has_started(self):
        """Return if the trajectory interpolator has started generating
        reference points.
        """

        return self._traj_interpolator.has_started()

    def has_finished(self):
        """Return `True` if the trajectory has finished."""
        return self._traj_interpolator.has_finished()

    def update_vehicle_pose(self, pos, quat):
        """Update the vehicle's pose information.
        
        > *Input arguments*
        
        * `pos` (*type:* `numpy.array`): Position vector
        * `quat` (*type:* `numpy.array`): Quaternion as `(qx, qy, qz, qw)`
        """
        if self._vehicle_pose is None:
            self._vehicle_pose = uuv_trajectory_generator.TrajectoryPoint()
        self._vehicle_pose.pos = pos
        self._vehicle_pose.rotq = quat
        self._vehicle_pose.t = rospy.get_time()
        self.init_odom_event.set()

    def get_vehicle_rot(self):
        """Return the vehicle's rotation quaternion."""
        self.init_odom_event.wait()
        return self._vehicle_pose.rotq

    def _update_trajectory_from_msg(self, msg):
        self._stamp_trajectory_received = rospy.get_time()
        self._traj_interpolator.init_from_trajectory_message(msg)
        self._logger.info('New trajectory received at ' + str(self._stamp_trajectory_received) + 's')
        self._update_trajectory_info()

    def start_station_keeping(self):
        """Start station keeping mode by setting the pose
        set-point of the vehicle as the last pose before the 
        vehicle finished automatic mode.
        """
        if self._vehicle_pose is not None:
            self._this_ref_pnt = deepcopy(self._vehicle_pose)
            self._this_ref_pnt.vel = np.zeros(6)
            self._this_ref_pnt.acc = np.zeros(6)
            self.set_station_keeping(True)
            self.set_automatic_mode(False)
            self._smooth_approach_on = False

    def hold_vehicle(self, request):
        """Service callback function to hold the vehicle's 
        current position.
        """
        if self._vehicle_pose is None:
            self._logger.error('Current pose of the vehicle is invalid')
            return HoldResponse(False)
        self.start_station_keeping()
        return HoldResponse(True)

    def start_waypoint_list(self, request):
        """Service callback function to follow a set of waypoints
        
        > *Input arguments*

        * `request` (*type:* `uuv_control_msgs.InitWaypointSet`)
        """
        if len(request.waypoints) == 0:
            self._logger.error('Waypoint list is empty')
            return InitWaypointSetResponse(False)
        t = rospy.Time(request.start_time.data.secs, request.start_time.data.nsecs)
        if t.to_sec() < rospy.get_time() and not request.start_now:
            self._logger.error('The trajectory starts in the past, correct the starting time!')
            return InitWaypointSetResponse(False)
        else:
            self._logger.info('Start waypoint trajectory now!')
        self._lock.acquire()
        # Create a waypoint set
        wp_set = uuv_waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)
        # Create a waypoint set message, to fill wp_set
        waypointset_msg = WaypointSet()
        waypointset_msg.header.stamp = rospy.get_time()
        waypointset_msg.header.frame_id = self.inertial_frame_id
        if request.start_now:
            waypointset_msg.start_time = rospy.get_time()
        else:
            waypointset_msg.start_time = t.to_sec()
        waypointset_msg.waypoints = request.waypoints
        wp_set.from_message(waypointset_msg)
        wp_set = self._transform_waypoint_set(wp_set)
        wp_set = self._apply_workspace_constraints(wp_set)

        if self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._station_keeping_center = None
            self._traj_interpolator.set_start_time((t.to_sec() if not request.start_now else rospy.get_time()))
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_circle_center = None
            self._smooth_approach_on = True
            self._logger.info('============================')
            self._logger.info('      WAYPOINT SET          ')
            self._logger.info('============================')
            self._logger.info('Interpolator = ' + request.interpolator.data)
            self._logger.info('# waypoints = %d' % self._traj_interpolator.get_waypoints().num_waypoints)
            self._logger.info('Starting time = %.2f' % (t.to_sec() if not request.start_now else rospy.get_time()))
            self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
            self._logger.info('============================')
            self._lock.release()
            return InitWaypointSetResponse(True)
        else:
            self._logger.error('Error occurred while parsing waypoints')
            self._lock.release()
            return InitWaypointSetResponse(False)

    def start_circle(self, request):
        """Service callback function to initialize a parametrized 
        circular trajectory.
        
        > *Input arguments*
        
        * `request` (*type:* `uuv_control_msgs.InitCircularTrajectory`)
        """
        if request.max_forward_speed <= 0 or request.radius <= 0 or \
           request.n_points <= 0:
            self._logger.error('Invalid parameters to generate a circular trajectory')
            return InitCircularTrajectoryResponse(False)
        t = rospy.Time(request.start_time.data.secs, request.start_time.data.nsecs)
        if t.to_sec() < rospy.get_time() and not request.start_now:
            self._logger.error('The trajectory starts in the past, correct the starting time!')
            return InitCircularTrajectoryResponse(False)
        try:
            wp_set = uuv_waypoints.WaypointSet(
                inertial_frame_id=self.inertial_frame_id)
            success = wp_set.generate_circle(radius=request.radius,
                                             center=request.center,
                                             num_points=request.n_points,
                                             max_forward_speed=request.max_forward_speed,
                                             theta_offset=request.angle_offset,
                                             heading_offset=request.heading_offset)
            if not success:
                self._logger.error('Error generating circular trajectory from waypoint set')
                return InitCircularTrajectoryResponse(False)
            wp_set = self._apply_workspace_constraints(wp_set)
            if wp_set.is_empty:
                self._logger.error('Waypoints violate workspace constraints, are you using world or world_ned as reference?')
                return InitCircularTrajectoryResponse(False)

            self._lock.acquire()
            # Activates station keeping
            self.set_station_keeping(True)
            self._traj_interpolator.set_interp_method('cubic')
            self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot())
            self._station_keeping_center = None
            self._traj_interpolator.set_start_time((t.to_sec() if not request.start_now else rospy.get_time()))
            if request.duration > 0:
                if self._traj_interpolator.set_duration(request.duration):
                    self._logger.info('Setting a maximum duration, duration=%.2f s' % request.duration)
                else:
                    self._logger.error('Setting maximum duration failed')
            self._update_trajectory_info()
            # Disables station keeping to start trajectory
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_circle_center = None
            self._smooth_approach_on = True

            self._logger.info('============================')
            self._logger.info('CIRCULAR TRAJECTORY GENERATED FROM WAYPOINT INTERPOLATION')
            self._logger.info('============================')
            self._logger.info('Radius [m] = %.2f' % request.radius)
            self._logger.info('Center [m] = (%.2f, %.2f, %.2f)' % (request.center.x, request.center.y, request.center.z))
            self._logger.info('# of points = %d' % request.n_points)
            self._logger.info('Max. forward speed = %.2f' % request.max_forward_speed)
            self._logger.info('Circle angle offset = %.2f' % request.angle_offset)
            self._logger.info('Heading offset = %.2f' % request.heading_offset)
            self._logger.info('# waypoints = %d' % self._traj_interpolator.get_waypoints().num_waypoints)
            self._logger.info('Starting from = ' + str(self._traj_interpolator.get_waypoints().get_waypoint(0).pos))
            self._logger.info('Starting time [s] = %.2f' % (t.to_sec() if not request.start_now else rospy.get_time()))
            self._logger.info('============================')
            self._lock.release()
            return InitCircularTrajectoryResponse(True)
        except Exception as e:
            self._logger.error('Error while setting circular trajectory, msg={}'.format(e))
            self.set_station_keeping(True)
            self.set_automatic_mode(False)
            self.set_trajectory_running(False)
            self._lock.release()
            return InitCircularTrajectoryResponse(False)

    def start_helix(self, request):
        """Service callback function to initialize a parametrized helical
        trajectory.
        
        > *Input arguments*
        
        * `request` (*type:* `uuv_control_msgs.InitHelicalTrajectory`)
        """
        if request.radius <= 0 or request.n_points <= 0 or \
           request.n_turns <= 0:
           self._logger.error('Invalid parameters to generate a helical trajectory')
           return InitHelicalTrajectoryResponse(False)
        t = rospy.Time(request.start_time.data.secs, request.start_time.data.nsecs)
        if t.to_sec() < rospy.get_time() and not request.start_now:
            self._logger.error('The trajectory starts in the past, correct the starting time!')
            return InitHelicalTrajectoryResponse(False)
        else:
            self._logger.info('Start helical trajectory now!')

        try:
            wp_set = uuv_waypoints.WaypointSet(inertial_frame_id=self.inertial_frame_id)
            success = wp_set.generate_helix(radius=request.radius,
                                            center=request.center,
                                            num_points=request.n_points,
                                            max_forward_speed=request.max_forward_speed,
                                            delta_z=request.delta_z,
                                            num_turns=request.n_turns,
                                            theta_offset=request.angle_offset,
                                            heading_offset=request.heading_offset)

            if not success:
                self._logger.error('Error generating circular trajectory from waypoint set')
                return InitHelicalTrajectoryResponse(False)
            wp_set = self._apply_workspace_constraints(wp_set)
            if wp_set.is_empty:
                self._logger.error('Waypoints violate workspace constraints, are you using world or world_ned as reference?')
                return InitHelicalTrajectoryResponse(False)

            self._lock.acquire()
            self.set_station_keeping(True)
            self._traj_interpolator.set_interp_method('cubic')
            if not self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
                self._logger.error('Error setting the waypoints')
                return InitHelicalTrajectoryResponse(False)

            self._station_keeping_center = None
            self._traj_interpolator.set_start_time((t.to_sec() if not request.start_now else rospy.get_time()))

            if request.duration > 0:
                if self._traj_interpolator.set_duration(request.duration):
                    self._logger.info('Setting a maximum duration, duration=%.2f s' % request.duration)
                else:
                    self._logger.error('Setting maximum duration failed')
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_circle_center = None
            self._smooth_approach_on = True

            self._logger.info('============================')
            self._logger.info('HELICAL TRAJECTORY GENERATED FROM WAYPOINT INTERPOLATION')
            self._logger.info('============================')
            self._logger.info('Radius [m] = %.2f' % request.radius)
            self._logger.info('Center [m] = (%.2f, %.2f, %.2f)' % (request.center.x, request.center.y, request.center.z))
            self._logger.info('# of points = %d' % request.n_points)
            self._logger.info('Max. forward speed = %.2f' % request.max_forward_speed)
            self._logger.info('Delta Z = %.2f' % request.delta_z)
            self._logger.info('# of turns = %d' % request.n_turns)
            self._logger.info('Helix angle offset = %.2f' % request.angle_offset)
            self._logger.info('Heading offset = %.2f' % request.heading_offset)
            self._logger.info('# waypoints = %d' % self._traj_interpolator.get_waypoints().num_waypoints)
            self._logger.info('Starting from = ' + str(self._traj_interpolator.get_waypoints().get_waypoint(0).pos))
            self._logger.info('Starting time [s] = %.2f' % (t.to_sec() if not request.start_now else rospy.get_time()))
            self._logger.info('============================')
            self._lock.release()
            return InitHelicalTrajectoryResponse(True)
        except Exception as e:
            self._logger.error('Error while setting helical trajectory, msg={}'.format(e))
            self.set_station_keeping(True)
            self.set_automatic_mode(False)
            self.set_trajectory_running(False)
            self._lock.release()
            return InitHelicalTrajectoryResponse(False)

    def init_waypoints_from_file(self, request):
        """Service callback function to initialize the path interpolator
        with a set of waypoints loaded from a YAML file.
        
        > *Input arguments*
        
        * `request` (*type:* `uuv_control_msgs.InitWaypointsFromFile`)
        """
        if (len(request.filename.data) == 0 or
                not isfile(request.filename.data)):
            self._logger.error('Invalid waypoint file')
            return InitWaypointsFromFileResponse(False)
        t = rospy.Time(request.start_time.data.secs, request.start_time.data.nsecs)
        if t.to_sec() < rospy.get_time() and not request.start_now:
            self._logger.error('The trajectory starts in the past, correct the starting time!')
            return InitWaypointsFromFileResponse(False)
        else:
            self._logger.info('Start waypoint trajectory now!')
        self._lock.acquire()
        self.set_station_keeping(True)
        self._traj_interpolator.set_interp_method(request.interpolator.data)

        wp_set = uuv_waypoints.WaypointSet()
        if not wp_set.read_from_file(request.filename.data):
            self._logger.info('Error occurred while parsing waypoint file')
            return InitWaypointsFromFileResponse(False)
        wp_set = self._transform_waypoint_set(wp_set)
        wp_set = self._apply_workspace_constraints(wp_set)

        if self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._station_keeping_center = None
            self._traj_interpolator.set_start_time((t.to_sec() if not request.start_now else rospy.get_time()))
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._idle_circle_center = None
            self._smooth_approach_on = True

            self._logger.info('============================')
            self._logger.info('IMPORT WAYPOINTS FROM FILE')
            self._logger.info('============================')
            self._logger.info('Filename = ' + request.filename.data)
            self._logger.info('Interpolator = ' + request.interpolator.data)
            self._logger.info('# waypoints = %d' % self._traj_interpolator.get_waypoints().num_waypoints)
            self._logger.info('Starting time = %.2f' % (t.to_sec() if not request.start_now else rospy.get_time()))
            self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
            self._logger.info('============================')
            self._lock.release()
            return InitWaypointsFromFileResponse(True)
        else:
            self._logger.error('Error occurred while parsing waypoint file')
            self._lock.release()
            return InitWaypointsFromFileResponse(False)

    def go_to(self, request):
        """Service callback function to initialize to set one target
        waypoint .
        
        > *Input arguments*
        
        * `request` (*type:* `uuv_control_msgs.GoTo`)
        """
        if self._vehicle_pose is None:
            self._logger.error('Current pose has not been initialized yet')
            return GoToResponse(False)
        if request.waypoint.max_forward_speed <= 0.0:
            self._logger.error('Max. forward speed must be greater than zero')
            return GoToResponse(False)
        self.set_station_keeping(True)
        self._lock.acquire()
        wp_set = uuv_waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)

        init_wp = uuv_waypoints.Waypoint(
            x=self._vehicle_pose.pos[0],
            y=self._vehicle_pose.pos[1],
            z=self._vehicle_pose.pos[2],
            max_forward_speed=request.waypoint.max_forward_speed,
            heading_offset=euler_from_quaternion(self.get_vehicle_rot())[2],
            use_fixed_heading=request.waypoint.use_fixed_heading,
            inertial_frame_id=self.inertial_frame_id)
        wp_set.add_waypoint(init_wp)
        wp_set.add_waypoint_from_msg(request.waypoint)
        wp_set = self._transform_waypoint_set(wp_set)
        self._traj_interpolator.set_interp_method(request.interpolator)
        if not self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._logger.error('Error while setting waypoints')
            self._lock.release()
            return GoToResponse(False)

        self._station_keeping_center = None
        t = rospy.get_time()
        self._traj_interpolator.set_start_time(t)
        self._update_trajectory_info()
        self.set_station_keeping(False)
        self.set_automatic_mode(True)
        self.set_trajectory_running(True)
        self._idle_circle_center = None
        self._smooth_approach_on = False

        self._logger.info('============================')
        self._logger.info('GO TO')
        self._logger.info('============================')
        self._logger.info('Heading offset [rad] = %.2f' % request.waypoint.heading_offset)
        self._logger.info('# waypoints = %d' % self._traj_interpolator.get_waypoints().num_waypoints)
        self._logger.info('Starting from = ' + str(self._traj_interpolator.get_waypoints().get_waypoint(0).pos))
        self._logger.info('Start time [s] = %.2f ' % t)
        self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
        self._logger.info('============================')
        self._lock.release()
        return GoToResponse(True)

    def go_to_incremental(self, request):
        """Service callback to set the command to the vehicle to move to a
        relative position in the world.

        > *Input arguments*

        * `request` (*type:* `uuv_control_msgs.GoToIncremental`)
        """
        if self._vehicle_pose is None:
            self._logger.error('Current pose has not been initialized yet')
            return GoToIncrementalResponse(False)
        if request.max_forward_speed <= 0:
            self._logger.error('Max. forward speed must be positive')
            return GoToIncrementalResponse(False)

        self._lock.acquire()
        self.set_station_keeping(True)
        wp_set = uuv_waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)
        init_wp = uuv_waypoints.Waypoint(
            x=self._vehicle_pose.pos[0],
            y=self._vehicle_pose.pos[1],
            z=self._vehicle_pose.pos[2],
            max_forward_speed=request.max_forward_speed,
            heading_offset=euler_from_quaternion(self.get_vehicle_rot())[2],
            inertial_frame_id=self.inertial_frame_id)
        wp_set.add_waypoint(init_wp)

        wp = uuv_waypoints.Waypoint(
            x=self._vehicle_pose.pos[0] + request.step.x,
            y=self._vehicle_pose.pos[1] + request.step.y,
            z=self._vehicle_pose.pos[2] + request.step.z,
            max_forward_speed=request.max_forward_speed,
            inertial_frame_id=self.inertial_frame_id)
        wp_set.add_waypoint(wp)

        self._traj_interpolator.set_interp_method(request.interpolator)
        if not self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot()):
            self._logger.error('Error while setting waypoints')
            self._lock.release()
            return GoToIncrementalResponse(False)

        self._station_keeping_center = None
        self._traj_interpolator.set_start_time(rospy.Time.now().to_sec())
        self._update_trajectory_info()
        self.set_station_keeping(False)
        self.set_automatic_mode(True)
        self.set_trajectory_running(True)
        self._idle_circle_center = None
        self._smooth_approach_on = False

        self._logger.info('============================')
        self._logger.info('GO TO INCREMENTAL')
        self._logger.info('============================')
        self._logger.info(str(wp_set))
        self._logger.info('# waypoints = %d' % wp_set.num_waypoints)
        self._logger.info('Inertial frame ID = ' + self.inertial_frame_id)
        self._logger.info('============================')
        self._lock.release()
        return GoToIncrementalResponse(True)

    def generate_reference(self, t):
        """Return a trajectory point computed by the interpolator for the 
        timestamp `t`, in case the vehicle is on `automatic` mode. In case 
        it is in station keeping, the pose is kept constant.
        
        > *Input arguments*
        
        * `t` (*type:* `float`): Timestamp
        
        > *Returns*
        
        `uuv_trajectory_generator.TrajectoryPoint`: Trajectory point
        """
        pnt = self._traj_interpolator.generate_reference(t, self._vehicle_pose.pos, self.get_vehicle_rot())
        if pnt is None:
            return self._vehicle_pose
        else:
            return pnt

    def get_idle_circle_path(self, n_points, radius=30):
        """Generate a waypoint set starting from the current 
        position of the vehicle in the shape of a circle to 
        initialize an AUVs idle mode.
        
        > *Input arguments*
        
        * `n_points` (*type:* `int`): Number of waypoints
        * `radius` (*type:* `float`): Circle radius in meters
        
        > *Returns*
        
        `uuv_waypoints.WaypointSet`: Set of waypoints for idle mode
        """
        pose = deepcopy(self._vehicle_pose)
        if self._idle_circle_center is None:
            frame = np.array([
                [np.cos(pose.rot[2]), -np.sin(pose.rot[2]), 0],
                [np.sin(pose.rot[2]), np.cos(pose.rot[2]), 0],
                [0, 0, 1]])
            self._idle_circle_center = (pose.pos + 0.8 * self._max_forward_speed * frame[:, 0].flatten()) + radius * frame[:, 1].flatten()
            self._idle_z = pose.pos[2]

        phi = lambda u: 2 * np.pi * u + pose.rot[2] - np.pi / 2
        u = lambda angle: (angle - pose.rot[2] + np.pi / 2) / (2 * np.pi)

        vec = pose.pos - self._idle_circle_center
        vec /= np.linalg.norm(vec)
        u_init = u(np.arctan2(vec[1], vec[0]))

        wp_set = uuv_waypoints.WaypointSet(
            inertial_frame_id=self.inertial_frame_id)

        for i in np.linspace(u_init, u_init + 1, n_points):
            wp = uuv_waypoints.Waypoint(
                x=self._idle_circle_center[0] + radius * np.cos(phi(i)),
                y=self._idle_circle_center[1] + radius * np.sin(phi(i)),
                z=self._idle_z,
                max_forward_speed=0.8 * self._max_forward_speed,
                inertial_frame_id=self.inertial_frame_id)
            wp_set.add_waypoint(wp)
        return wp_set

    def interpolate(self, t):
        """Function interface to the controller. Calls the interpolator to
        calculate the current trajectory sample or returns a fixed position
        based on the past odometry measurements for station keeping.

        > *Input arguments*
        
        * `t` (*type:* `float`): Timestamp

        > *Returns*
        
        `uuv_trajectory_generator.TrajectoryPoint`: Trajectory point
        """

        self._lock.acquire()
        if not self._station_keeping_on and self._traj_running:
            if self._smooth_approach_on:
                # Generate extra waypoint before the initial waypoint
                self._calc_smooth_approach()
                self._smooth_approach_on = False
                self._update_trajectory_info()
                time.sleep(0.5)
                self._logger.info('Adding waypoints to approach the given waypoint trajectory')

            # Get interpolated reference from the reference trajectory
            self._this_ref_pnt = self._traj_interpolator.interpolate(t, self._vehicle_pose.pos, self.get_vehicle_rot())

            if self._look_ahead_delay > 0:
                self._this_ref_pnt = self.generate_reference(t + self._look_ahead_delay)

            self._max_time_pub.publish(Float64(self._traj_interpolator.get_max_time() - rospy.get_time()))

            if not self._traj_running:
                self._traj_running = True
                self._logger.info(rospy.get_namespace() + ' - Trajectory running')

            if self._traj_running and (self._traj_interpolator.has_finished() or self._station_keeping_on):
                # Trajectory ended, start station keeping mode
                self._logger.info(rospy.get_namespace() + ' - Trajectory completed!')
                if self._this_ref_pnt is None:
                    # TODO Fix None value coming from the odometry
                    if self._is_teleop_active:
                        self._this_ref_pnt = self._calc_teleop_reference()
                    else:
                        self._this_ref_pnt = deepcopy(self._vehicle_pose)
                self._this_ref_pnt.vel = np.zeros(6)
                self._this_ref_pnt.acc = np.zeros(6)
                self._start_count_idle = rospy.get_time()
                self.set_station_keeping(True)
                self.set_automatic_mode(False)
                self.set_trajectory_running(False)
        elif self._this_ref_pnt is None:
            self._traj_interpolator.set_interp_method('lipb')
            # Use the latest position and heading of the vehicle from the odometry to enter station keeping mode
            if self._is_teleop_active:
                self._this_ref_pnt = self._calc_teleop_reference()
            else:
                self._this_ref_pnt = deepcopy(self._vehicle_pose)
            # Set roll and pitch reference to zero
            yaw = self._this_ref_pnt.rot[2]
            self._this_ref_pnt.rot = [0, 0, yaw]
            self.set_automatic_mode(False)
        elif self._station_keeping_on:
            if self._is_teleop_active:
                self._this_ref_pnt = self._calc_teleop_reference()
            self._max_time_pub.publish(Float64(0))
            #######################################################################
            if not self._thrusters_only and not self._is_teleop_active and rospy.get_time() - self._start_count_idle > self._timeout_idle_mode:
                self._logger.info('AUV STATION KEEPING')
                if self._station_keeping_center is None:
                    self._station_keeping_center = self._this_ref_pnt

                wp_set = self.get_idle_circle_path(20, self._idle_radius)
                wp_set = self._apply_workspace_constraints(wp_set)
                if wp_set.is_empty:
                    raise rospy.ROSException('Waypoints violate workspace constraints, are you using world or world_ned as reference?')

                # Activates station keeping
                self.set_station_keeping(True)
                self._traj_interpolator.set_interp_method('cubic')
                self._traj_interpolator.set_waypoints(wp_set, self.get_vehicle_rot())
                self._traj_interpolator.set_start_time(rospy.get_time())
                self._update_trajectory_info()
                # Disables station keeping to start trajectory
                self.set_station_keeping(False)
                self.set_automatic_mode(True)
                self.set_trajectory_running(True)
                self._smooth_approach_on = False
            #######################################################################
        self._lock.release()
        return self._this_ref_pnt
