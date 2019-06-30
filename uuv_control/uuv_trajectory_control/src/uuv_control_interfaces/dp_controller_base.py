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
from copy import deepcopy
import numpy as np
import rospy
import logging
import sys
import tf

from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped, \
    Vector3, Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from uuv_control_interfaces.vehicle import Vehicle
from tf_quaternion.transformations import euler_from_quaternion, \
    quaternion_multiply, quaternion_matrix, quaternion_conjugate, \
    quaternion_inverse
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint
from uuv_control_msgs.srv import *
from uuv_auv_control_allocator.msg import AUVCommand

from .dp_controller_local_planner import DPControllerLocalPlanner as LocalPlanner
from ._log import get_logger


class DPControllerBase(object):
    """General abstract class for DP controllers for underwater vehicles.
    This is an abstract class, must be inherited by a controller module that
    overrides the update_controller method. If the controller is set to be
    model based (is_model_based=True), than the vehicle parameters are going
    to be read from the ROS parameter server.

    > *Input arguments*

    * `is_model_based` (*type:* `bool`, *default:* `False`): If `True`, the
    controller uses a model of the vehicle, `False` if it is non-model-based.
    * `list_odometry_callbacks` (*type:* `list`, *default:* `None`): List of 
    function handles or `lambda` functions that will be called after each 
    odometry update.
    * `planner_full_dof` (*type:* `bool`, *default:* `False`): Set the local
    planner to generate 6 DoF trajectories. Otherwise, the planner will only
    generate 4 DoF trajectories `(x, y, z, yaw)`.

    > *ROS parameters*

    * `use_stamped_poses_only` (*type:* `bool`, *default:* `False`): If `True`,
    the reference path will be generated with stamped poses only, velocity
    and acceleration references are set to zero.
    * `thrusters_only` (*type:* `bool`, *default:* `True`): If `True`, the
    vehicle only has thrusters as actuators and a thruster manager node is
    running and the control output will be published as `geometry_msgs/WrenchStamped`. 
    If `False`, the AUV force allocation should be used to compute 
    the control output for each actuator and the output control will be 
    generated as a `uuv_auv_control_allocator.AUVCommand` message.
    * `saturation` (*type:* `float`, *default:* `5000`): Absolute saturation
    of the control signal. 
    * `use_auv_control_allocator` (*type:* `bool`, *default:* `False`): If `True`,
    the output control will be `AUVCommand` message.
    * `min_thrust` (*type:* `float`, *default:* `40`): Min. thrust set-point output,
    (parameter only valid for AUVs).

    > *ROS publishers*

    * `thruster_output` (*message:* `geometry_msgs/WrenchStamped`): Control set-point
    for the thruster manager node (requirement: ROS parameters must be `thrusters_only`
    must be set as `True` and a thruster manager from `uuv_thruster_manager` node must 
    be running).
    * `auv_command_output` (*message:* `uuv_auv_control_allocator.AUVCommand`): Control
    set-point for the AUV allocation node (requirement: ROS parameters must be 
    `thrusters_only` must be set as `False` and a AUV control allocation node from  
    `uuv_auv_control_allocator` node must be running).
    * `reference` (*message:* `uuv_control_msgs/TrajectoryPoint`): Current reference 
    trajectory point.
    * `error` (*message:* `uuv_control_msgs/TrajectoryPoint`): Current trajectory error.

    > *ROS services*

    * `reset_controller` (*service:* `uuv_control_msgs/ResetController`): Reset all 
    variables, including error and reference signals.
    """

    _LABEL = ''

    def __init__(self, is_model_based=False, list_odometry_callbacks=None,
        planner_full_dof=False):
        # Flag will be set to true when all parameters are initialized correctly
        self._is_init = False
        self._logger = get_logger()
        
        # Reading current namespace
        self._namespace = rospy.get_namespace()

        # Configuration for the vehicle dynamic model
        self._is_model_based = is_model_based

        if self._is_model_based:
            self._logger.info('Setting controller as model-based')
        else:
            self._logger.info('Setting controller as non-model-based')

        self._use_stamped_poses_only = False
        if rospy.has_param('~use_stamped_poses_only'):
            self._use_stamped_poses_only = rospy.get_param('~use_stamped_poses_only')

        # Flag indicating if the vehicle has only thrusters, otherwise
        # the AUV allocation node will be used
        self.thrusters_only = rospy.get_param('~thrusters_only', True)

        # Instance of the local planner for local trajectory generation
        self._local_planner = LocalPlanner(
            full_dof=planner_full_dof,
            stamped_pose_only=self._use_stamped_poses_only,
            thrusters_only=self.thrusters_only)

        self._control_saturation = 5000
        # TODO: Fix the saturation term and how it is applied
        if rospy.has_param('~saturation'):
            self._thrust_saturation = rospy.get_param('~saturation')
            if self._control_saturation <= 0:
                raise rospy.ROSException('Invalid control saturation forces')

        # Flag indicating either use of the AUV control allocator or
        # direct command of fins and thruster
        self.use_auv_control_allocator = False
        if not self.thrusters_only:
            self.use_auv_control_allocator = rospy.get_param(
                '~use_auv_control_allocator', False)

        # Remap the following topics, if needed
        # Publisher for thruster allocator
        if self.thrusters_only:
            self._thrust_pub = rospy.Publisher(
                'thruster_output', WrenchStamped, queue_size=1)
        else:
            self._thrust_pub = None

        if not self.thrusters_only:
            self._auv_command_pub = rospy.Publisher(
                'auv_command_output', AUVCommand, queue_size=1)
        else:
            self._auv_command_pub = None

        self._min_thrust = rospy.get_param('~min_thrust', 40.0)

        self._reference_pub = rospy.Publisher('reference',
                                              TrajectoryPoint,
                                              queue_size=1)
        # Publish error (for debugging)
        self._error_pub = rospy.Publisher('error',
                                          TrajectoryPoint, queue_size=1)

        self._init_reference = False

        # Reference with relation to the INERTIAL frame
        self._reference = dict(pos=np.zeros(3),
                               rot=np.zeros(4),
                               vel=np.zeros(6),
                               acc=np.zeros(6))

        # Errors wih relation to the BODY frame
        self._errors = dict(pos=np.zeros(3),
                            rot=np.zeros(4),
                            vel=np.zeros(6))

        # Time step
        self._dt = 0
        self._prev_time = rospy.get_time()

        self._services = dict()
        self._services['reset'] = rospy.Service('reset_controller',
                                                ResetController,
                                                self.reset_controller_callback)

        # Time stamp for the received trajectory
        self._stamp_trajectory_received = rospy.get_time()

        # Instance of the vehicle model
        self._vehicle_model = None
        # If list of callbacks is empty, set the default
        if list_odometry_callbacks is not None and \
            isinstance(list_odometry_callbacks, list):
            self._odometry_callbacks = list_odometry_callbacks
        else:
            self._odometry_callbacks = [self.update_errors,
                                        self.update_controller]
        # Initialize vehicle, if model based
        self._create_vehicle_model()
        # Flag to indicate that odometry topic is receiving data
        self._init_odom = False

        # Subscribe to odometry topic
        self._odom_topic_sub = rospy.Subscriber(
            'odom', numpy_msg(Odometry), self._odometry_callback)

        # Stores last simulation time
        self._prev_t = -1.0
        self._logger.info('DP controller successfully initialized')

    def __del__(self):
        # Removing logging message handlers
        while self._logger.handlers:
            self._logger.handlers.pop()

    @staticmethod
    def get_controller(name, *args):
        """Create instance of a specific DP controller."""
        for controller in DPControllerBase.__subclasses__():
            if name == controller.__name__:
                self._logger.info('Creating controller={}'.format(name))
                return controller(*args)

    @staticmethod
    def get_list_of_controllers():
        """Return list of DP controllers using this interface."""
        return [controller.__name__ for controller in
                DPControllerBase.__subclasses__()]

    @property
    def label(self):
        """`str`: Identifier name of the controller"""
        return self._LABEL

    @property
    def odom_is_init(self):
        """`bool`: `True` if the first odometry message was received"""
        return self._init_odom

    @property
    def error_pos_world(self):
        """`numpy.array`: Position error wrt world frame"""
        return np.dot(self._vehicle_model.rotBtoI, self._errors['pos'])

    @property
    def error_orientation_quat(self):
        """`numpy.array`: Orientation error"""
        return deepcopy(self._errors['rot'][0:3])

    @property
    def error_orientation_rpy(self):
        """`numpy.array`: Orientation error in Euler angles."""
        e1 = self._errors['rot'][0]
        e2 = self._errors['rot'][1]
        e3 = self._errors['rot'][2]
        eta = self._errors['rot'][3]
        rot = np.array([[1 - 2 * (e2**2 + e3**2),
                         2 * (e1 * e2 - e3 * eta),
                         2 * (e1 * e3 + e2 * eta)],
                        [2 * (e1 * e2 + e3 * eta),
                         1 - 2 * (e1**2 + e3**2),
                         2 * (e2 * e3 - e1 * eta)],
                        [2 * (e1 * e3 - e2 * eta),
                         2 * (e2 * e3 + e1 * eta),
                         1 - 2 * (e1**2 + e2**2)]])
        # Roll
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        # Pitch, treating singularity cases
        den = np.sqrt(1 - rot[2, 1]**2)
        pitch = - np.arctan(rot[2, 1] / max(0.001, den))
        # Yaw
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        return np.array([roll, pitch, yaw])

    @property
    def error_pose_euler(self):
        """`numpy.array`: Pose error with orientation represented in Euler angles."""
        return np.hstack((self._errors['pos'], self.error_orientation_rpy))

    @property
    def error_vel_world(self):
        """`numpy.array`: Linear velocity error"""
        return np.dot(self._vehicle_model.rotBtoI, self._errors['vel'])

    def __str__(self):
        msg = 'Dynamic positioning controller\n'
        msg += 'Controller= ' + self._LABEL + '\n'
        msg += 'Is model based? ' + str(self._is_model_based) + '\n'
        msg += 'Vehicle namespace= ' + self._namespace
        return msg

    def _create_vehicle_model(self):
        """Create a new instance of a vehicle model. If controller is not model
        based, this model will have its parameters set to 0 and will be used
        to receive and transform the odometry data.
        """
        if self._vehicle_model is not None:
            del self._vehicle_model
        self._vehicle_model = Vehicle(
            inertial_frame_id=self._local_planner.inertial_frame_id)

    def _update_reference(self):
        """Call the local planner interpolator to retrieve a trajectory 
        point and publish the reference message as `uuv_control_msgs/TrajectoryPoint`.
        """
        # Update the local planner's information about the vehicle's pose
        self._local_planner.update_vehicle_pose(
            self._vehicle_model.pos, self._vehicle_model.quat)

        t = rospy.get_time()
        reference = self._local_planner.interpolate(t)

        if reference is not None:
            self._reference['pos'] = reference.p
            self._reference['rot'] = reference.q
            self._reference['vel'] = np.hstack((reference.v, reference.w))
            self._reference['acc'] = np.hstack((reference.a, reference.alpha))
        if reference is not None and self._reference_pub.get_num_connections() > 0:
            # Publish current reference
            msg = TrajectoryPoint()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self._local_planner.inertial_frame_id
            msg.pose.position = Vector3(*self._reference['pos'])
            msg.pose.orientation = Quaternion(*self._reference['rot'])
            msg.velocity.linear = Vector3(*self._reference['vel'][0:3])
            msg.velocity.angular = Vector3(*self._reference['vel'][3:6])
            msg.acceleration.linear = Vector3(*self._reference['acc'][0:3])
            msg.acceleration.angular = Vector3(*self._reference['acc'][3:6])
            self._reference_pub.publish(msg)
        return True

    def _update_time_step(self):
        """Update time step."""
        t = rospy.get_time()
        self._dt = t - self._prev_time
        self._prev_time = t

    def _reset_controller(self):
        """Reset reference and and error vectors."""
        self._init_reference = False

        # Reference with relation to the INERTIAL frame
        self._reference = dict(pos=np.zeros(3),
                               rot=np.zeros(4),
                               vel=np.zeros(6),
                               acc=np.zeros(6))

        # Errors wih relation to the BODY frame
        self._errors = dict(pos=np.zeros(3),
                            rot=np.zeros(4),
                            vel=np.zeros(6))

    def reset_controller_callback(self, request):
        """Service handler function."""
        self._reset_controller()
        return ResetControllerResponse(True)

    def update_controller(self):
        """This function must be implemented by derived classes
        with the implementation of the control algorithm.
        """
        # Does nothing, must be overloaded
        raise NotImplementedError()

    def update_errors(self):
        """Update error vectors."""
        if not self.odom_is_init:
            self._logger.warning('Odometry topic has not been update yet')
            return
        self._update_reference()
        # Calculate error in the BODY frame
        self._update_time_step()
        # Rotation matrix from INERTIAL to BODY frame
        rotItoB = self._vehicle_model.rotItoB
        rotBtoI = self._vehicle_model.rotBtoI
        if self._dt > 0:
            # Update position error with respect to the the BODY frame
            pos = self._vehicle_model.pos
            vel = self._vehicle_model.vel
            quat = self._vehicle_model.quat
            self._errors['pos'] = np.dot(
                rotItoB, self._reference['pos'] - pos)

            # Update orientation error
            self._errors['rot'] = quaternion_multiply(
                quaternion_inverse(quat), self._reference['rot'])

            # Velocity error with respect to the the BODY frame
            self._errors['vel'] = np.hstack((
                np.dot(rotItoB, self._reference['vel'][0:3]) - vel[0:3],
                np.dot(rotItoB, self._reference['vel'][3:6]) - vel[3:6]))

        if self._error_pub.get_num_connections() > 0:
            stamp = rospy.Time.now()
            msg = TrajectoryPoint()
            msg.header.stamp = stamp
            msg.header.frame_id = self._local_planner.inertial_frame_id
            # Publish pose error
            msg.pose.position = Vector3(*np.dot(rotBtoI, self._errors['pos']))
            msg.pose.orientation = Quaternion(*self._errors['rot'])
            # Publish velocity errors in INERTIAL frame
            msg.velocity.linear = Vector3(*np.dot(rotBtoI, self._errors['vel'][0:3]))
            msg.velocity.angular = Vector3(*np.dot(rotBtoI, self._errors['vel'][3:6]))
            self._error_pub.publish(msg)

    def publish_control_wrench(self, force):
        """Publish the thruster manager control set-point.
        
        > *Input arguments*
        
        * `force` (*type:* `numpy.array`): 6 DoF control 
        set-point wrench vector
        """
        if not self.odom_is_init:
            return

        # Apply saturation
        for i in range(6):
            if force[i] < -self._control_saturation:
                force[i] = -self._control_saturation
            elif force[i] > self._control_saturation:
                force[i] = self._control_saturation

        if not self.thrusters_only:
            surge_speed = self._vehicle_model.vel[0]
            self.publish_auv_command(surge_speed, force)
            return

        force_msg = WrenchStamped()
        force_msg.header.stamp = rospy.Time.now()
        force_msg.header.frame_id = '%s/%s' % (self._namespace, self._vehicle_model.body_frame_id)
        force_msg.wrench.force.x = force[0]
        force_msg.wrench.force.y = force[1]
        force_msg.wrench.force.z = force[2]

        force_msg.wrench.torque.x = force[3]
        force_msg.wrench.torque.y = force[4]
        force_msg.wrench.torque.z = force[5]

        self._thrust_pub.publish(force_msg)

    def publish_auv_command(self, surge_speed, wrench):
        """Publish the AUV control command message
        
        > *Input arguments*
        
        * `surge_speed` (*type:* `float`): Reference surge speed
        * `wrench` (*type:* `numpy.array`): 6 DoF wrench vector
        """
        if not self.odom_is_init:
            return

        surge_speed = max(0, surge_speed)

        msg = AUVCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '%s/%s' % (self._namespace, self._vehicle_model.body_frame_id)
        msg.surge_speed = surge_speed
        msg.command.force.x = max(self._min_thrust, wrench[0])
        msg.command.force.y = wrench[1]
        msg.command.force.z = wrench[2]
        msg.command.torque.x = wrench[3]
        msg.command.torque.y = wrench[4]
        msg.command.torque.z = wrench[5]

        self._auv_command_pub.publish(msg)

    def _odometry_callback(self, msg):
        """Odometry topic subscriber callback function.
        
        > *Input arguments*

        * `msg` (*type:* `nav_msgs/Odometry`): Input odometry 
        message
        """
        self._vehicle_model.update_odometry(msg)

        if not self._init_odom:
            self._init_odom = True        

        if len(self._odometry_callbacks):
            for func in self._odometry_callbacks:
                func()
