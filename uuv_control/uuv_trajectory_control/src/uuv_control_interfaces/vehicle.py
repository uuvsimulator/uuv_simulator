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
from nav_msgs.msg import Odometry
from copy import deepcopy
from rospy.numpy_msg import numpy_msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion, \
    quaternion_matrix, rotation_matrix, is_same_transform


def cross_product_operator(x):
    """Return a cross product operator for the given vector."""
    S = np.array([[0, -x[2], x[1]],
                  [x[2], 0, -x[0]],
                  [-x[1], x[0], 0]])
    return S


class Vehicle(object):

    """Vehicle interface to be used by model-based controllers. It receives the
    parameters necessary to compute the vehicle's motion according to Fossen's.
    """

    INSTANCE = None

    def __init__(self, list_odometry_callbacks=list(), sub_to_odometry=True):
        """Class constructor."""
        # Reading current namespace
        self._namespace = rospy.get_namespace()

        # Load the list of callback function handles to be called in the
        # odometry callback
        self._list_callbacks = list_odometry_callbacks

        self._mass = 0
        if rospy.has_param('~mass'):
            self._mass = rospy.get_param('~mass')
            if self._mass <= 0:
                raise rospy.ROSException('Mass has to be positive')

        self._inertial = dict(ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)
        if rospy.has_param('~inertial'):
            print 'has inertial'
            inertial = rospy.get_param('~inertial')
            for key in self._inertial:
                if key not in inertial:
                    raise rospy.ROSException('Invalid moments of inertia')
            self._inertial = inertial

        self._cog = [0, 0, 0]
        if rospy.has_param('~cog'):
            self._cog = rospy.get_param('~cog')
            if len(self._cog) != 3:
                raise rospy.ROSException('Invalid center of gravity vector')

        self._cob = [0, 0, 0]
        if rospy.has_param('~cog'):
            self._cob = rospy.get_param('~cob')
            if len(self._cob) != 3:
                raise rospy.ROSException('Invalid center of buoyancy vector')

        self._body_frame = 'base_link'
        if rospy.has_param('~base_link'):
            self._body_frame = rospy.get_param('~base_link')

        self._volume = 0.0
        if rospy.has_param('~volume'):
            self._volume = rospy.get_param('~volume')
            if self._volume <= 0:
                raise rospy.ROSException('Invalid volume')

        # Fluid density
        self._density = 1028.0
        if rospy.has_param('~density'):
            self._density = rospy.get_param('~density')
            if self._density <= 0:
                raise rospy.ROSException('Invalid fluid density')

        # Bounding box
        self._height = 0.0
        self._length = 0.0
        self._width = 0.0
        if rospy.has_param('~height'):
            self._height = rospy.get_param('~height')
            if self._height <= 0:
                raise rospy.ROSException('Invalid height')

        if rospy.has_param('~length'):
            self._length = rospy.get_param('~length')
            if self._length <= 0:
                raise rospy.ROSException('Invalid length')

        if rospy.has_param('~width'):
            self._width = rospy.get_param('~width')
            if self._width <= 0:
                raise rospy.ROSException('Invalid width')


        # Calculating the rigid-body mass matrix
        self._M = np.zeros(shape=(6, 6), dtype=float)
        self._M[0:3, 0:3] = self._mass * np.eye(3)
        self._M[0:3, 3:6] = - self._mass * \
            cross_product_operator(self._cog)
        self._M[3:6, 0:3] = self._mass * \
            cross_product_operator(self._cog)
        self._M[3:6, 3:6] = self._calc_inertial_tensor()

        # Loading the added-mass matrix
        self._Ma = np.zeros((6, 6))
        if rospy.has_param('~Ma'):
            self._Ma = np.array(rospy.get_param('~Ma'))
            if self._Ma.shape != (6, 6):
                raise rospy.ROSException('Invalid added mass matrix')

        # Sum rigid-body and added-mass matrices
        self._Mtotal = np.zeros(shape=(6, 6))
        self._calc_mass_matrix()

        # Acceleration of gravity
        self._gravity = 9.81

        # Initialize the Coriolis and centripetal matrix
        self._C = np.zeros((6, 6))

        # Vector of restoring forces
        self._g = np.zeros(6)

        # Loading the linear damping coefficients
        self._linear_damping = np.zeros(shape=(6, 6))
        if rospy.has_param('~linear_damping'):
            self._linear_damping = np.array(rospy.get_param('~linear_damping'))
            if self._linear_damping.shape == (6,):
                self._linear_damping = np.diag(self._linear_damping)
            if self._linear_damping.shape != (6, 6):
                raise rospy.ROSException('Linear damping must be given as a 6x6 matrix or the diagonal coefficients')

        # Loading the nonlinear damping coefficients
        self._quad_damping = np.zeros(shape=(6,))
        if rospy.has_param('~quad_damping'):
            self._quad_damping = np.array(rospy.get_param('~quad_damping'))
            if self._quad_damping.shape != (6,):
                raise rospy.ROSException('Quadratic damping must be given defined with 6 coefficients')

        # Loading the linear damping coefficients proportional to the forward speed
        self._linear_damping_forward_speed = np.zeros(shape=(6, 6))
        if rospy.has_param('~linear_damping_forward_speed'):
            self._linear_damping_forward_speed = np.array(rospy.get_param('~linear_damping_forward_speed'))
            if self._linear_damping_forward_speed.shape == (6,):
                self._linear_damping_forward_speed = np.diag(self._linear_damping_forward_speed)
            if self._linear_damping_forward_speed.shape != (6, 6):
                raise rospy.ROSException('Linear damping proportional to the forward speed must be given as a 6x6 '
                                         'matrix or the diagonal coefficients')

        # Initialize damping matrix
        self._D = np.zeros((6, 6))

        # Vehicle states
        self._pose = dict(pos=np.zeros(3),
                          rot=quaternion_from_euler(0, 0, 0))
        # Velocity in the body frame
        self._vel = np.zeros(6)
        # Acceleration in the body frame
        self._acc = np.zeros(6)
        # Generalized forces
        self._gen_forces = np.zeros(6)
        # Flag to indicate that odometry topic is receiving data
        self._init_odom = False
        if sub_to_odometry:
            # Subscribe to odometry topic
            self._odom_topic_sub = rospy.Subscriber(
                'odom', numpy_msg(Odometry), self._odometry_callback)
        else:
            self._init_odom = True
            self._odom_topic_sub = None

    @property
    def namespace(self):
        """Return robot namespace."""
        return self._namespace

    @property
    def odom_is_init(self):
        return self._init_odom

    @property
    def mass(self):
        return self._mass

    @property
    def height(self):
        return self._height

    @property
    def width(self):
        return self._width

    @property
    def length(self):
        return self._length

    @property
    def pos(self):
        """Return the position of the vehicle."""
        return deepcopy(self._pose['pos'])

    @pos.setter
    def pos(self, position):
        pos = np.array(position)
        if pos.size != 3:
            print 'Invalid position vector'
        else:
            self._pose['pos'] = pos

    @property
    def depth(self):
        """Return depth of the vehicle."""
        return deepcopy(np.abs(self._pose['pos'][2]))

    @property
    def heading(self):
        """Return the heading of the vehicle."""
        return deepcopy(self.euler[2])

    @property
    def quat(self):
        """Return orientation quaternion."""
        return deepcopy(self._pose['rot'])

    @quat.setter
    def quat(self, q):
        q_rot = np.array(q)
        if q_rot.size != 4:
            print 'Invalid quaternion'
        else:
            self._pose['rot'] = q_rot

    @property
    def quat_dot(self):
        """Return the time derivative of the quaternion vector."""
        return np.dot(self.TBtoIquat, self.vel[3:6])

    @property
    def vel(self):
        """Return linear and angular velocity vector."""
        return deepcopy(self._vel)

    @vel.setter
    def vel(self, velocity):
        """Set the velocity vector in the BODY frame."""
        v = np.array(velocity)
        if v.size != 6:
            print 'Invalid velocity vector'
        else:
            self._vel = v

    @property
    def acc(self):
        """Return linear and angular acceleration vector."""
        return deepcopy(self._acc)

    @property
    def euler(self):
        """Return orientation in Euler angles as described in Fossen, 2011."""
        # Rotation matrix from BODY to INERTIAL
        rot = self.rotBtoI
        # Roll
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        # Pitch, treating singularity cases
        den = np.sqrt(1 - rot[2, 1]**2)
        pitch = - np.arctan(rot[2, 1] / max(0.001, den))
        # Yaw
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        return roll, pitch, yaw

    @property
    def euler_dot(self):
        """Return time derivative of the Euler angles."""
        return np.dot(self.TItoBeuler, self.vel[3:6])

    @property
    def restoring_forces(self):
        """Return the restoring force vector."""
        self._update_restoring()
        return deepcopy(self._g)

    @property
    def Mtotal(self):
        return deepcopy(self._Mtotal)

    @property
    def Ctotal(self):
        return deepcopy(self._C)

    @property
    def Dtotal(self):
        return deepcopy(self._D)

    @property
    def pose_euler(self):
        """Return pose as a vector, orientation in Euler angles."""
        roll, pitch, yaw = self.euler
        pose = np.zeros(6)
        pose[0:3] = self.pos
        pose[3] = roll
        pose[4] = pitch
        pose[5] = yaw
        return pose

    @property
    def pose_quat(self):
        """Return pose as a vector, orientation as quaternion."""
        pose = np.zeros(7)
        pose[0:3] = self.pos
        pose[3:7] = self.quat
        return pose

    @property
    def rotItoB(self):
        """Return rotation from INERTIAL to BODY frame"""
        return self.rotBtoI.T

    @property
    def rotBtoI(self):
        """Return rotation from BODY to INERTIAL frame using the zyx convention
           to retrieve Euler angles from the quaternion vector (Fossen, 2011).
        """
        # Using the (x, y, z, w) format to describe quaternions
        e1 = self._pose['rot'][0]
        e2 = self._pose['rot'][1]
        e3 = self._pose['rot'][2]
        eta = self._pose['rot'][3]
        R = np.array([[1 - 2 * (e2**2 + e3**2),
                       2 * (e1 * e2 - e3 * eta),
                       2 * (e1 * e3 + e2 * eta)],
                      [2 * (e1 * e2 + e3 * eta),
                       1 - 2 * (e1**2 + e3**2),
                       2 * (e2 * e3 - e1 * eta)],
                      [2 * (e1 * e3 - e2 * eta),
                       2 * (e2 * e3 + e1 * eta),
                       1 - 2 * (e1**2 + e2**2)]])
        return R

    @property
    def TItoBeuler(self):
        r, p, y = self.euler
        T = np.array([[1, 0, -np.sin(p)],
                      [0, np.cos(r), np.cos(p) * np.sin(r)],
                      [0, -np.sin(r), np.cos(p) * np.cos(r)]])
        return T

    @property
    def TBtoIeuler(self):
        r, p, y = self.euler
        cp = np.cos(p)
        cp = np.sign(cp) * min(0.001, np.abs(cp))
        T = 1 / cp * np.array(
            [[0, np.sin(r) * np.sin(p), np.cos(r) * np.sin(p)],
             [0, np.cos(r) * np.cos(p), -np.cos(p) * np.sin(r)],
             [0, np.sin(r), np.cos(r)]])
        return T

    @property
    def TBtoIquat(self):
        """
        Return matrix for transformation of BODY-fixed angular velocities in the
        BODY frame in relation to the INERTIAL frame into quaternion rate.
        """
        e1 = self._pose['rot'][0]
        e2 = self._pose['rot'][1]
        e3 = self._pose['rot'][2]
        eta = self._pose['rot'][3]
        T = 0.5 * np.array(
            [[-e1, -e2, -e3],
             [eta, -e3, e2],
             [e3, eta, -e1],
             [-e2, e1, eta]]
        )
        return T

    def to_SNAME(self, x):
        try:
            if x.shape == (3,):
                return np.array([x[0], -1 * x[1], -1 * x[2]])
            elif x.shape == (6,):
                return np.array([x[0], -1 * x[1], -1 * x[2],
                                 x[3], -1 * x[4], -1 * x[5]])
        except:
            print('Invalid input vector, v=' + str(x))
            return None

    def from_SNAME(self, x):
        return self.to_SNAME(x)

    def print_info(self):
        """Print the vehicle's parameters."""
        print 'Namespace: {}'.format(self._namespace)
        print 'Mass: {0:.3f} kg'.format(self._mass)
        print 'System inertia matrix:\n', self._M
        print 'Added-mass:\n', self._Ma
        print 'M:\n', self._Mtotal
        print 'Linear damping:', self._linear_damping
        print 'Quad. damping:', self._quad_damping
        print 'Center of gravity:', self._cog
        print 'Center of buoyancy:', self._cob
        print 'Inertial:\n', self._calc_inertial_tensor()

    def _calc_mass_matrix(self):
        self._Mtotal = self._M + self._Ma

    def _update_coriolis(self, vel=None):
        if vel is not None:
            if vel.shape != (6,):
                raise rospy.ROSException('Velocity vector has the wrong '
                                         'dimension')
            nu = vel
        else:
            nu = self.to_SNAME(self._vel)

        self._C = np.zeros((6, 6))

        S_12 = - cross_product_operator(
            np.dot(self._Mtotal[0:3, 0:3], nu[0:3]) +
            np.dot(self._Mtotal[0:3, 3:6], nu[3:6]))
        S_22 = - cross_product_operator(
            np.dot(self._Mtotal[3:6, 0:3], nu[0:3]) +
            np.dot(self._Mtotal[3:6, 3:6], nu[3:6]))

        self._C[0:3, 3:6] = S_12
        self._C[3:6, 0:3] = S_12
        self._C[3:6, 3:6] = S_22

    def _update_damping(self, vel=None):
        if vel is not None:
            if vel.shape != (6,):
                raise rospy.ROSException('Velocity vector has the wrong '
                                         'dimension')
            # Assume the input velocity is already given in the SNAME convention
            nu = vel
        else:
            nu = self.to_SNAME(self._vel)

        self._D = -1 * self._linear_damping - nu[0] * self._linear_damping_forward_speed
        for i in range(6):
            self._D[i, i] += -1 * self._quad_damping[i] * np.abs(nu[i])

    def _calc_inertial_tensor(self):
        return np.array(
            [[self._inertial['ixx'], self._inertial['ixy'],
              self._inertial['ixz']],
             [self._inertial['ixy'], self._inertial['iyy'],
              self._inertial['iyz']],
             [self._inertial['ixz'], self._inertial['iyz'],
              self._inertial['izz']]])

    def _update_restoring(self, use_sname=False):
        """
        Update the restoring forces for the current orientation.
        """
        Fg = np.array([0, 0, -self._mass * self._gravity])
        Fb = np.array([0, 0, self._volume * self._gravity * self._density])
        self._g = np.zeros(6)

        self._g[0:3] = -1 * np.dot(self.rotItoB, Fg + Fb)
        self._g[3:6] = -1 * np.dot(self.rotItoB,
                                   np.cross(self._cog, Fg) + np.cross(self._cob, Fb))

        if use_sname:
            self._g = self.to_SNAME(self._g)

    def set_added_mass(self, Ma):
        """Set added-mass matrix coefficients."""
        if Ma.shape != (6, 6):
            print "Added mass matrix must have dimensions 6x6"
            return False
        self._Ma = np.array(Ma, copy=True)
        self._calc_mass_matrix()
        return True

    def set_damping_coef(self, linear_damping, quad_damping):
        """Set linear and quadratic damping coefficients."""
        if linear_damping.size != 6 or quad_damping.size != 6:
            print 'Invalid dimensions for damping coefficient vectors'
            return False
        self._linear_damping = np.array(linear_damping, copy=True)
        self._quad_damping = np.array(quad_damping, copy=True)
        return True

    def compute_force(self, acc=None, vel=None, with_restoring=True, use_sname=True):
        """Return the sum of forces acting on the vehicle.

        Given acceleration and velocity vectors, this function returns the
        sum of forces given the rigid-body and hydrodynamic models for the
        marine vessel.
        """
        if acc is not None:
            if acc.shape != (6,):
                raise rospy.ROSException('Acceleration vector must have 6 '
                                         'elements')
            # It is assumed the input acceleration is given in the SNAME convention
            nu_dot = acc
        else:
            # Convert the acceleration vector to the SNAME convention since the odometry is usually given using the
            # ENU convention
            nu_dot = self.to_SNAME(self._acc)

        if vel is not None:
            if vel.shape != (6,):
                raise rospy.ROSException('Velocity vector must have 6 '
                                         'elements')
            # It is assumed the input velocity is given in the SNAME convention
            nu = vel
        else:
            nu = self.to_SNAME(self._vel)

        self._update_damping(nu)
        self._update_coriolis(nu)
        self._update_restoring(use_sname=True)

        if with_restoring:
            g = deepcopy(self._g)
        else:
            g = np.zeros(6)

        f = np.dot(self._Mtotal, nu_dot) + np.dot(self._C, nu) + \
            np.dot(self._D, nu) + g

        if not use_sname:
            f = self.from_SNAME(f)

        return f

    def compute_acc(self, gen_forces=None, use_sname=True):
        """Calculate inverse dynamics to obtain the acceleration vector."""
        self._gen_forces = np.zeros(shape=(6,))
        if gen_forces is not None:
            # It is assumed the generalized forces are given in the SNAME convention
            self._gen_forces = gen_forces
        # Check if the mass and inertial parameters were set
        if self._Mtotal.sum() == 0:
            self._acc = np.zeros(6)
        else:
            nu = self.to_SNAME(self._vel)

            self._update_damping()
            self._update_coriolis()
            self._update_restoring(use_sname=True)
            # Compute the vehicle's acceleration
            self._acc = np.linalg.solve(self._Mtotal, self._gen_forces -
                                        np.dot(self._C, nu) -
                                        np.dot(self._D, nu) -
                                        self._g)
        if not use_sname:
            self._acc = self.from_SNAME(self._acc)

        return self._acc

    def get_jacobian(self):
        """
        Return the Jacobian for the current orientation using transformations
        from BODY to INERTIAL frame.
        """
        jac = np.zeros(shape=(6, 6))
        # Build the Jacobian matrix
        jac[0:3, 0:3] = self.rotBtoI
        jac[3:6, 3:6] = self.TBtoIeuler
        return jac

    def add_odometry_callback(self, callback):
        self._list_callbacks.append(callback)

    def _odometry_callback(self, msg):
        """Odometry topic subscriber callback function."""
        # The frames of reference delivered by the odometry seems to be as
        # follows
        # position -> world frame
        # orientation -> world frame
        # linear velocity -> world frame
        # angular velocity -> world frame

        # Update the velocity vector
        # Update the pose in the inertial frame
        self._pose['pos'] = np.array([msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      msg.pose.pose.position.z])

        # Using the (x, y, z, w) format for quaternions
        self._pose['rot'] = np.array([msg.pose.pose.orientation.x,
                                      msg.pose.pose.orientation.y,
                                      msg.pose.pose.orientation.z,
                                      msg.pose.pose.orientation.w])
        # Linear velocity on the INERTIAL frame
        lin_vel = np.array([msg.twist.twist.linear.x,
                            msg.twist.twist.linear.y,
                            msg.twist.twist.linear.z])
        # Transform linear velocity to the BODY frame
        lin_vel = np.dot(self.rotItoB, lin_vel)
        # Angular velocity in the INERTIAL frame
        ang_vel = np.array([msg.twist.twist.angular.x,
                            msg.twist.twist.angular.y,
                            msg.twist.twist.angular.z])
        # Transform angular velocity to BODY frame
        ang_vel = np.dot(self.rotItoB, ang_vel)
        # Store velocity vector
        self._vel = np.hstack((lin_vel, ang_vel))

        if not self._init_odom:
            self._init_odom = True

        if len(self._list_callbacks):
            for func in self._list_callbacks:
                func()
            try:
                for func in self._list_callbacks:
                    func()
            except:
                print 'Invalid callback function handle'
