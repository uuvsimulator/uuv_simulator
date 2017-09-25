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

from copy import deepcopy

import numpy as np
import rospy
import logging
import sys
from geometry_msgs.msg import Wrench, PoseStamped, TwistStamped, Vector3, \
    Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from uuv_control_interfaces.vehicle import Vehicle
from tf.transformations import euler_from_quaternion, \
    quaternion_multiply, quaternion_matrix, quaternion_conjugate, \
    quaternion_inverse
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint
from uuv_control_msgs.srv import *
from .dp_controller_local_planner import DPControllerLocalPlanner as LocalPlanner


class DPControllerBase(object):
    """General abstract class for DP controllers for underwater vehicles.
    This is an abstract class, must be inherited by a controller module that
    overrides the update_controller method. If the controller is set to be
    model based (is_model_based=True), than the vehicle parameters are going
    to be read from the ROS parameter server.
    """

    _LABEL = ''

    def __init__(self, is_model_based=False, list_odometry_callbacks=[],
        planner_full_dof=False):
        # Flag will be set to true when all parameters are initialized correctly
        self._is_init = False
        self._logger = logging.getLogger('dp_controller')
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        self._logger.addHandler(out_hdlr)
        self._logger.setLevel(logging.INFO)

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

        # Instance of the local planner for local trajectory generation
        self._local_planner = LocalPlanner(full_dof=planner_full_dof,
                                           stamped_pose_only=self._use_stamped_poses_only)

        # Instance of the vehicle model
        self._vehicle_model = None
        # If list of callbacks is empty, set the default
        if len(list_odometry_callbacks):
            self._odometry_callbacks = list_odometry_callbacks
        else:
            self._odometry_callbacks = [self.update_errors,
                                        self.update_controller]
        # Initialize vehicle, if model based
        self._create_vehicle_model()

        self._control_saturation = 5000

        if rospy.has_param('~saturation'):
            self._thrust_saturation = rospy.get_param('~saturation')
            if self._control_saturation <= 0:
                raise rospy.ROSException('Invalid control saturation forces')

        # Remap the following topics, if needed
        # Publisher for thruster allocator
        self._thrust_pub = rospy.Publisher('thruster_output',
                                           Wrench, queue_size=1)

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
                print 'Creating controller=', name
                return controller(*args)

    @staticmethod
    def get_list_of_controllers():
        """Return list of DP controllers using this interface."""
        return [controller.__name__ for controller in
                DPControllerBase.__subclasses__()]

    @property
    def label(self):
        return self._LABEL

    @property
    def odom_is_init(self):
        return self._vehicle_model.odom_is_init

    @property
    def error_pos_world(self):
        return np.dot(self._vehicle_model.rotBtoI, self._errors['pos'])

    @property
    def error_orientation_quat(self):
        return deepcopy(self._errors['rot'])

    @property
    def error_orientation_rpy(self):
        """Return orientation error in Euler angles."""
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
        """Pose error with orientation represented in Euler angles."""
        return np.hstack((self._errors['pos'], self.error_orientation_rpy))

    @property
    def error_vel_world(self):
        return np.dot(self._vehicle_model.rotBtoI, self._errors['vel'])

    def __str__(self):
        msg = 'Dynamic positioning controller\n'
        msg += 'Controller= ' + self._LABEL + '\n'
        msg += 'Is model based? ' + str(self._is_model_based) + '\n'
        msg += 'Vehicle namespace= ' + self._namespace
        return msg

    def _create_vehicle_model(self):
        """
        Create a new instance of a vehicle model. If controller is not model
        based, this model will have its parameters set to 0 and will be used
        to receive and transform the odometry data.
        """
        if self._vehicle_model is not None:
            del self._vehicle_model
        self._vehicle_model = Vehicle(self._odometry_callbacks)

    def _update_reference(self):
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

            # Publish current reference
            msg = TrajectoryPoint()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'world'
            msg.pose.position = Vector3(*self._reference['pos'])
            msg.pose.orientation = Quaternion(*self._reference['rot'])
            msg.velocity.linear = Vector3(*self._reference['vel'][0:3])
            msg.velocity.angular = Vector3(*self._reference['vel'][3:6])
            msg.acceleration.linear = Vector3(*self._reference['acc'][0:3])
            msg.acceleration.angular = Vector3(*self._reference['acc'][3:6])
            self._reference_pub.publish(msg)
        return True

    def _update_time_step(self):
        t = rospy.get_time()
        self._dt = t - self._prev_time
        self._prev_time = t

    def _reset_controller(self):
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
        self._reset_controller()
        return ResetControllerResponse(True)

    def update_controller(self):
        # Does nothing, must be overloaded
        raise NotImplementedError()

    def update_errors(self):
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

            # Update orientation error with respect to the BODY frame
            self._errors['rot'] = quaternion_multiply(
                quaternion_inverse(quat), self._reference['rot'])

            # Velocity error with respect to the the BODY frame
            self._errors['vel'] = np.hstack((
                np.dot(rotItoB, self._reference['vel'][0:3]) - vel[0:3],
                np.dot(rotItoB, self._reference['vel'][3:6]) - vel[3:6]))

        stamp = rospy.Time.now()
        msg = TrajectoryPoint()
        msg.header.stamp = stamp
        msg.header.frame_id = 'world'
        # Publish pose error
        msg.pose.position = Vector3(*np.dot(rotBtoI, self._errors['pos']))
        msg.pose.orientation = Quaternion(*self._errors['rot'])
        # Publish velocity errors in INERTIAL frame
        msg.velocity.linear = Vector3(*np.dot(rotBtoI, self._errors['vel'][0:3]))
        msg.velocity.angular = Vector3(*np.dot(rotBtoI, self._errors['vel'][3:6]))
        self._error_pub.publish(msg)

    def publish_control_wrench(self, force):
        if not self.odom_is_init:
            return
        # Apply saturation
        for i in range(6):
            if force[i] < -self._control_saturation:
                force[i] = -self._control_saturation
            elif force[i] > self._control_saturation:
                force[i] = self._control_saturation

        force_msg = Wrench()
        force_msg.force.x = force[0]
        force_msg.force.y = force[1]
        force_msg.force.z = force[2]

        force_msg.torque.x = force[3]
        force_msg.torque.y = force[4]
        force_msg.torque.z = force[5]

        self._thrust_pub.publish(force_msg)
