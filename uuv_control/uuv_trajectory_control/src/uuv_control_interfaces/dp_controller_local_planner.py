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
from os.path import isfile
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from uuv_control_msgs.srv import *
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint, WaypointSet
import uuv_trajectory_generator
import logging
import sys
from tf.transformations import quaternion_about_axis, quaternion_multiply, quaternion_inverse


class DPControllerLocalPlanner(object):
    """
    Local planner for the dynamic positioning controllers to interpolate trajectories and generate trajectories from
    interpolated waypoint paths.
    """

    def __init__(self, full_dof=False, stamped_pose_only=False):
        self._logger = logging.getLogger('dp_local_planner')
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        self._logger.addHandler(out_hdlr)
        self._logger.setLevel(logging.INFO)

        self._traj_interpolator = uuv_trajectory_generator.TrajectoryGenerator(
            full_dof=full_dof, stamped_pose_only=stamped_pose_only)

        # Max. allowed forward speed
        self._max_forward_speed = 1.0

        if rospy.has_param('~max_forward_speed'):
            speed = rospy.get_param('~max_forward_speed')
            if speed >= 0:
                self._max_forward_speed = speed
            else:
                self._logger.error('Invalid max. forward speed, keeping 1 m/s')
        
        self._logger.info('Max. forward speed = ' + str(self._max_forward_speed))

        # dt used to compute the pose reference from the joystick input
        self._dt = 0.0
        # Time stamp for the last velocity reference received
        self._last_teleop_update = None
        # Flag to indicate if the teleoperation node is active
        self._is_teleop_active = False
        # Teleop node twist message
        self._teleop_vel_ref = None

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
                                                
        self._teleop_sub = rospy.Subscriber('cmd_vel', Twist, self._update_teleop)

        self._waypoints_msg = None
        self._trajectory_msg = None

        # Publishing topic for the trajectory given to the controller
        self._input_trajectory_sub = rospy.Subscriber(
            'input_trajectory', Trajectory, self._update_trajectory_from_msg)

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

    def _publish_trajectory_info(self, event):
        """
        Publish messages for the waypoints, trajectory and internal flags.
        """
        if self._waypoints_msg is not None:
            self._waypoints_pub.publish(self._waypoints_msg)
        if self._trajectory_msg is not None:
            self._trajectory_pub.publish(self._trajectory_msg)
        self._station_keeping_pub.publish(Bool(self._station_keeping_on))
        self._automatic_control_pub.publish(Bool(self._is_automatic))
        self._traj_tracking_pub.publish(Bool(self._traj_running))
        return True

    def _update_trajectory_info(self):
        self._waypoints_msg = WaypointSet()
        if self._traj_interpolator.is_using_waypoints():
            wps = self._traj_interpolator.get_waypoints()
            if wps is not None:
                self._waypoints_msg = wps.to_message()
        msg = self._traj_interpolator.get_trajectory_as_message()
        if msg is not None:
            self._trajectory_msg = msg
            self._logger.info('Updating the trajectory information')
        else:
            self._logger.error('Error generating trajectory message')

    def _update_teleop(self, msg):
        """
        Callback to the twist teleop subscriber.
        """
        # Test whether the vehicle is in automatic mode (following a given trajectory)
        if self._is_automatic:
            self._teleop_vel_ref = None
            return 

        # If this is the first twist message since the last time automatic mode was turned off,
        # then just update the teleop timestamp and wait for the next message to allow computing 
        # pose and velocity reference.
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
        """
        Compute pose and velocity reference using the joystick linear and angular velocity input.
        """
        # Check if there is already a timestamp for the last received reference message 
        # from the teleop node 
        if self._last_teleop_update is None:
            self._is_teleop_active = False

        # Compute time step
        self._dt = rospy.get_time() - self._last_teleop_update

        # Compute the pose and velocity reference if the computed time step is positive and
        # the twist teleop message is valid
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

            ref_pnt.rotq = quaternion_multiply(self._vehicle_pose.rotq, step.rotq)
            
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

    def _display_message(self, msg):
        print 'DP Local Planner - ' + str(msg)

    def _calc_smooth_approach(self):
        """
        Add the current vehicle position as waypoint to allow a smooth
        approach to the given trajectory.
        """
        if self._vehicle_pose is None:
            self._logger.error('Simulation not properly initialized yet, ignoring approach...')
            return
        if not self._traj_interpolator.is_using_waypoints():
            self._logger.error('Not using the waypoint interpolation method')
            return
        init_wp = uuv_trajectory_generator.Waypoint(
            x=self._vehicle_pose.pos[0],
            y=self._vehicle_pose.pos[1],
            z=self._vehicle_pose.pos[2],
            max_forward_speed=self._traj_interpolator.get_waypoints().get_waypoint(0).max_forward_speed)
        first_wp = self._traj_interpolator.get_waypoints().get_waypoint(0)

        dx = first_wp.x - init_wp.x
        dy = first_wp.y - init_wp.y
        dz = first_wp.z - init_wp.z

        # One new waypoint at each meter
        self._logger.info('Adding waypoints to approach the first position in the given waypoint set')
        steps = int(np.floor(first_wp.dist(init_wp.pos)) / 10)
        if steps > 0:
            for i in range(1, steps):
                wp = uuv_trajectory_generator.Waypoint(
                    x=first_wp.x - i * dx / steps,
                    y=first_wp.y - i * dy / steps,
                    z=first_wp.z - i * dz / steps,
                    max_forward_speed=self._traj_interpolator.get_waypoints().get_waypoint(0).max_forward_speed)
                self._traj_interpolator.add_waypoint(wp, add_to_beginning=True)
        self._traj_interpolator.add_waypoint(init_wp, add_to_beginning=True)
        self._update_trajectory_info()

    def is_station_keeping_on(self):
        return self._station_keeping_on

    def is_automatic_on(self):
        return self._is_automatic

    def set_station_keeping(self, is_on=True):
        """Set station keeping mode flag."""
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
        """
        Return if the trajectory interpolator has started generating reference
        points.
        """

        return self._traj_interpolator.has_started()

    def has_finished(self):
        return self._traj_interpolator.has_finished()

    def update_vehicle_pose(self, pos, quat):
        if self._vehicle_pose is None:
            self._vehicle_pose = uuv_trajectory_generator.TrajectoryPoint()
        self._vehicle_pose.pos = pos
        self._vehicle_pose.rotq = quat
        self._vehicle_pose.t = rospy.get_time()

    def _update_trajectory_from_msg(self, msg):
        self._stamp_trajectory_received = rospy.get_time()
        self._traj_interpolator.init_from_trajectory_message(msg)
        self._logger.info('New trajectory received at ' + str(self._stamp_trajectory_received) + 's')
        self._update_trajectory_info()

    def start_station_keeping(self):
        if self._vehicle_pose is not None:
            self._this_ref_pnt = deepcopy(self._vehicle_pose)
            self._this_ref_pnt.vel = np.zeros(6)
            self._this_ref_pnt.acc = np.zeros(6)
            self.set_station_keeping(True)
            self.set_automatic_mode(False)
            self._smooth_approach_on = False

    def hold_vehicle(self, request):
        """
        Service callback function to hold the vehicle's current position.
        """
        if self._vehicle_pose is None:
            self._logger.error('Current pose of the vehicle is invalid')
            return HoldResponse(False)
        self.start_station_keeping()
        return HoldResponse(True)

    def start_waypoint_list(self, request):
        if len(request.waypoints) == 0:
            self._logger.error('Waypoint list is empty')
            return InitWaypointSet(False)
        self.set_station_keeping(False)
        self.set_automatic_mode(True)
        self.set_trajectory_running(True)
        self._smooth_approach_on = True
        return InitWaypointSet(True)

    def start_circle(self, request):
        if request.max_forward_speed <= 0 or request.radius <= 0 or \
           request.n_points <= 0:
            self._logger.error('Invalid parameters to generate a circular trajectory')
            return InitCircularTrajectoryResponse(False)
        t = rospy.Time(request.start_time.data.secs, request.start_time.data.nsecs)
        if t.to_sec() < rospy.get_time() and not request.start_now:
            self._logger.error('The trajectory starts in the past, correct the starting time!')
            return InitCircularTrajectoryResponse(False)
        wp_set = uuv_trajectory_generator.WaypointSet()
        success = wp_set.generate_circle(radius=request.radius,
                                         center=request.center,
                                         num_points=request.n_points,
                                         max_forward_speed=request.max_forward_speed,
                                         theta_offset=request.angle_offset,
                                         heading_offset=request.heading_offset)
        if success:
            # Activates station keeping
            self.set_station_keeping(True)
            self._traj_interpolator.set_interp_method('cubic_interpolator')
            self._traj_interpolator.set_waypoints(wp_set)
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
            self._smooth_approach_on = True

            print '==========================================================='
            print 'CIRCULAR TRAJECTORY GENERATED FROM WAYPOINT INTERPOLATION'
            print '==========================================================='
            print 'Radius [m] =', request.radius
            print 'Center [m] = (%.2f, %.2f, %.2f)' % (request.center.x, request.center.y, request.center.z)
            print '# of points =', request.n_points
            print 'Max. forward speed =', request.max_forward_speed
            print 'Circle angle offset =', request.angle_offset
            print 'Heading offset =', request.heading_offset
            print '# waypoints =', self._traj_interpolator.get_waypoints().num_waypoints
            print 'Starting from =', self._traj_interpolator.get_waypoints().get_waypoint(0).pos
            print 'Starting time [s] =', (t.to_sec() if not request.start_now else rospy.get_time())
            print 'Estimated max. time [s] = ', self._traj_interpolator.get_max_time()
            print '==========================================================='
            return InitCircularTrajectoryResponse(True)
        else:
            self._logger.error('Error generating circular trajectory from waypoint set')
            return InitCircularTrajectoryResponse(False)

    def start_helix(self, request):
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
        wp_set = uuv_trajectory_generator.WaypointSet()
        success = wp_set.generate_helix(radius=request.radius,
                                        center=request.center,
                                        num_points=request.n_points,
                                        max_forward_speed=request.max_forward_speed,
                                        delta_z=request.delta_z,
                                        num_turns=request.n_turns,
                                        theta_offset=request.angle_offset,
                                        heading_offset=request.heading_offset)
        if success:
            self.set_station_keeping(True)
            self._traj_interpolator.set_interp_method('cubic_interpolator')
            self._traj_interpolator.set_waypoints(wp_set)
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
            self._smooth_approach_on = True

            print '==========================================================='
            print 'HELICAL TRAJECTORY GENERATED FROM WAYPOINT INTERPOLATION'
            print '==========================================================='
            print 'Radius [m] =', request.radius
            print 'Center [m] = (%.2f, %.2f, %.2f)' % (request.center.x, request.center.y, request.center.z)
            print '# of points =', request.n_points
            print 'Max. forward speed =', request.max_forward_speed
            print 'Delta Z =', request.delta_z
            print '# of turns =', request.n_turns
            print 'Helix angle offset =', request.angle_offset
            print 'Heading offset =', request.heading_offset
            print '# waypoints =', self._traj_interpolator.get_waypoints().num_waypoints
            print 'Starting from =', self._traj_interpolator.get_waypoints().get_waypoint(0).pos
            print 'Starting time [s] =', (t.to_sec() if not request.start_now else rospy.get_time())
            print 'Estimated max. time [s] = ', self._traj_interpolator.get_max_time()
            print '==========================================================='
            return InitHelicalTrajectoryResponse(True)
        else:
            self._logger.error('Error generating helical trajectory from waypoint set')
            return InitHelicalTrajectoryResponse(False)

    def init_waypoints_from_file(self, request):
        if (len(request.filename.data) == 0 or
                not isfile(request.filename.data)):
            self._logger.error('Invalid waypoint file')
            return InitWaypointsFromFileResponse(False)
        t = rospy.Time(request.start_time.data.secs, request.start_time.data.nsecs)
        if t.to_sec() < rospy.get_time() and not request.start_now:
            self._logger.error('The trajectory starts in the past, correct the starting time!')
            return InitHelicalTrajectoryResponse(False)
        else:
            self._logger.info('Start waypoint trajectory now!')
        self.set_station_keeping(True)
        self._traj_interpolator.set_interp_method('lipb_interpolator')
        if self._traj_interpolator.init_from_waypoint_file(request.filename.data):
            t = (t.to_sec() if not request.start_now else rospy.get_time())
            self._traj_interpolator.set_start_time(t)
            self._update_trajectory_info()
            self.set_station_keeping(False)
            self.set_automatic_mode(True)
            self.set_trajectory_running(True)
            self._smooth_approach_on = True

            print '==========================================================='
            print 'IMPORT WAYPOINTS FROM FILE'
            print '==========================================================='
            print '# waypoints =', self._traj_interpolator.get_waypoints().num_waypoints
            print 'Starting time =', t
            print 'Estimated max. time [s] = ', self._traj_interpolator.get_max_time()
            print '==========================================================='
            return InitWaypointsFromFileResponse(True)
        else:
            self._logger.info('Error occurred while parsing waypoint file')
            return InitWaypointsFromFileResponse(False)

    def go_to(self, request):
        if self._vehicle_pose is None:
            self._logger.error('Current pose has not been initialized yet')
            return GoToResponse(False)
        if request.waypoint.max_forward_speed <= 0.0:
            self._logger.error('Max. forward speed must be greater than zero')
            return GoToResponse(False) 
        self.set_station_keeping(True)
        wp_set = uuv_trajectory_generator.WaypointSet()

        init_wp = uuv_trajectory_generator.Waypoint(
            x=self._vehicle_pose.pos[0],
            y=self._vehicle_pose.pos[1],
            z=self._vehicle_pose.pos[2],
            max_forward_speed=request.waypoint.max_forward_speed,
            heading_offset=request.waypoint.heading_offset,
            use_fixed_heading=request.waypoint.use_fixed_heading)
        wp_set.add_waypoint(init_wp)
        wp_set.add_waypoint_from_msg(request.waypoint)
        self._traj_interpolator.set_interp_method('lipb_interpolator')
        if not self._traj_interpolator.set_waypoints(wp_set):
            self._logger.error('Error while setting waypoints')
            return GoToResponse(False)

        t = rospy.Time.now().to_sec()
        self._traj_interpolator.set_start_time(t)
        self._update_trajectory_info()
        self.set_station_keeping(False)
        self.set_automatic_mode(True)
        self.set_trajectory_running(True)
        self._smooth_approach_on = False

        print '==========================================================='
        print 'GO TO'
        print '==========================================================='
        print wp_set
        print 'Heading offset =', request.waypoint.heading_offset
        print '# waypoints =', self._traj_interpolator.get_waypoints().num_waypoints
        print 'Starting from =', self._traj_interpolator.get_waypoints().get_waypoint(0).pos
        print 'Start time [s]: ', t
        print 'Estimated max. time [s] = ', self._traj_interpolator.get_max_time()
        print '==========================================================='
        return GoToResponse(True)

    def go_to_incremental(self, request):
        """
        Service callback to set the command to the vehicle to move to a
        relative position in the world.
        """
        if self._vehicle_pose is None:
            self._logger.error('Current pose has not been initialized yet')
            return GoToIncrementalResponse(False)
        if request.max_forward_speed <= 0:
            self._logger.error('Max. forward speed must be positive')
            return GoToIncrementalResponse(False)
        self.set_station_keeping(True)
        wp_set = uuv_trajectory_generator.WaypointSet()
        init_wp = uuv_trajectory_generator.Waypoint(
            x=self._vehicle_pose.pos[0],
            y=self._vehicle_pose.pos[1],
            z=self._vehicle_pose.pos[2],
            max_forward_speed=request.max_forward_speed)
        wp_set.add_waypoint(init_wp)

        wp = uuv_trajectory_generator.Waypoint(
            x=self._vehicle_pose.pos[0] + request.step.x,
            y=self._vehicle_pose.pos[1] + request.step.y,
            z=self._vehicle_pose.pos[2] + request.step.z,
            max_forward_speed=request.max_forward_speed)
        wp_set.add_waypoint(wp)

        self._traj_interpolator.set_interp_method('lipb_interpolator')
        if not self._traj_interpolator.set_waypoints(wp_set):
            self._logger.error('Error while setting waypoints')
            return GoToIncrementalResponse(False)

        self._traj_interpolator.set_start_time(rospy.Time.now().to_sec())
        self._update_trajectory_info()
        self.set_station_keeping(False)
        self.set_automatic_mode(True)
        self.set_trajectory_running(True)
        self._smooth_approach_on = False

        print '==========================================================='
        print 'GO TO INCREMENTAL'
        print '==========================================================='
        print wp_set
        print '# waypoints =', wp_set.num_waypoints
        print '==========================================================='

        return GoToIncrementalResponse(True)
    
    def interpolate(self, t):
        """
        Function interface to the controller. Calls the interpolator to
        calculate the current trajectory sample or returns a fixed position
        based on the past odometry measurements for station keeping.
        """

        if not self._station_keeping_on and self._traj_running:
            if self._smooth_approach_on:
                # Generate extra waypoint before the initial waypoint
                self._calc_smooth_approach()
                self._smooth_approach_on = False
                self._logger.info('Adding waypoints to approach the given waypoint trajectory')
            # Get interpolated reference from the reference trajectory
            self._this_ref_pnt = self._traj_interpolator.interpolate(t)

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
                self.set_station_keeping(True)
                self.set_automatic_mode(False)
                self.set_trajectory_running(False)
        elif self._this_ref_pnt is None:
            self._traj_interpolator.set_interp_method('lipb_interpolator')
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
            
        return self._this_ref_pnt
