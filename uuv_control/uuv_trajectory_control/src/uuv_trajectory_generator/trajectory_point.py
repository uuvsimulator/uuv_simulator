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
import numpy as np
from uuv_control_msgs.msg import TrajectoryPoint as TrajectoryPointMsg
import geometry_msgs.msg as geometry_msgs
from tf_quaternion.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix


class TrajectoryPoint(object):
    """Trajectory point data structure.
    
    > *Input arguments*
    
    * `t` (*type:* `float`, *value:* `0`): Timestamp
    * `pos` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`): 
    3D position vector in meters
    * `quat` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0, 1]`): 
    Quaternion in the form of `(x, y, z, w)`.
    * `lin_vel` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`): 
    3D linear velocity vector in m/s
    * `ang_vel` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`): 
    3D angular velocity vector as rad/s
    * `lin_acc` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`): 
    3D linear acceleration vector as m/s$^2$
    * `ang_acc` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`): 
    3D angular acceleration vector as rad/s$^2$
    """
    def __init__(self, t=0.0, pos=[0, 0, 0], quat=[0, 0, 0, 1],
                 lin_vel=[0, 0, 0], ang_vel=[0, 0, 0], lin_acc=[0, 0, 0],
                 ang_acc=[0, 0, 0]):
        self._pos = np.array(pos)
        self._rot = np.array(quat)
        self._vel = np.hstack((lin_vel, ang_vel))
        self._acc = np.hstack((lin_acc, ang_acc))
        self._t = t

    def __str__(self):
        msg = 'Time [s] = {}\n'.format(self._t)
        msg += 'Position [m] = ({}, {}, {})\n'.format(self._pos[0], self._pos[1], self._pos[2])
        eu = [a * 180 / np.pi for a in self.rot]
        msg += 'Rotation [degrees] = ({}, {}, {})\n'.format(eu[0], eu[1], eu[2])
        msg += 'Lin. velocity [m/s] = ({}, {}, {})\n'.format(self._vel[0], self._vel[1], self._vel[2])
        msg += 'Ang. velocity [m/s] = ({}, {}, {})\n'.format(self._vel[3], self._vel[4], self._vel[5])
        return msg

    def __eq__(self, pnt):
        return self._t == pnt._t and np.array_equal(self._pos, pnt._pos) and \
            np.array_equal(self._rot, pnt._rot) and \
            np.array_equal(self._vel, pnt._vel) and \
            np.array_equal(self._acc, pnt._acc)

    @property
    def p(self):
        """`numpy.array`: Position vector"""
        return self._pos

    @property
    def q(self):
        """`numpy.array`: Quaternion vector as `(x, y, z, w)`"""
        return self._rot

    @property
    def v(self):
        """`numpy.array`: Linear velocity vector"""
        return self._vel[0:3]

    @property
    def w(self):
        """`numpy.array`: Angular velocity vector"""
        return self._vel[3::]

    @property
    def a(self):
        """`numpy.array`: Linear acceleration vector"""
        return self._acc[0:3]

    @property
    def alpha(self):
        """`numpy.array`: Angular acceleartion vector"""
        return self._acc[3::]

    @property
    def x(self):
        """`float`: X coordinate of position vector"""
        return self._pos[0]

    @x.setter
    def x(self, x):
        self._pos[0] = x

    @property
    def y(self):
        """`float`: Y coordinate of position vector"""
        return self._pos[1]

    @y.setter
    def y(self, y):
        self._pos[1] = y

    @property
    def z(self):
        """`float`: Z coordinate of position vector"""
        return self._pos[2]

    @z.setter
    def z(self, z):
        self._pos[2] = z

    @property
    def t(self):
        """`float`: Time stamp"""
        return self._t

    @t.setter
    def t(self, new_t):
        self._t = new_t

    @property
    def pos(self):
        """`numpy.array`: Position vector"""
        return self._pos

    @pos.setter
    def pos(self, new_pos):
        self._pos = np.array(new_pos)

    @property
    def rot(self):
        """`numpy.array`: `roll`, `pitch` and `yaw` angles"""
        rpy = euler_from_quaternion(self._rot)
        return np.array([rpy[0], rpy[1], rpy[2]])

    @rot.setter
    def rot(self, new_rot):
        self._rot = quaternion_from_euler(*new_rot)

    @property
    def rot_matrix(self):
        """`numpy.array`: Rotation matrix"""
        return quaternion_matrix(self._rot)[0:3, 0:3]

    @property
    def rotq(self):
        """`numpy.array`: Quaternion vector as `(x, y, z, w)`"""
        return self._rot

    @rotq.setter
    def rotq(self, quat):
        self._rot = quat

    @property
    def vel(self):
        """`numpy.array`: Linear velocity vector"""
        return self._vel

    @vel.setter
    def vel(self, new_vel):
        self._vel = np.array(new_vel)

    @property
    def acc(self):
        """`numpy.array`: Linear acceleration vector"""
        return self._acc

    @acc.setter
    def acc(self, new_acc):
        self._acc = np.array(new_acc)

    def to_message(self):
        """Convert current data to a trajectory point message.
        
        > *Returns*
        
        Trajectory point message as `uuv_control_msgs/TrajectoryPoint`
        """
        p_msg = TrajectoryPointMsg()
        # FIXME Sometimes the time t stored is NaN
        p_msg.header.stamp = rospy.Time(self.t)
        p_msg.pose.position = geometry_msgs.Vector3(*self.p)
        p_msg.pose.orientation = geometry_msgs.Quaternion(*self.q)
        p_msg.velocity.linear = geometry_msgs.Vector3(*self.v)
        p_msg.velocity.angular = geometry_msgs.Vector3(*self.w)
        p_msg.acceleration.linear = geometry_msgs.Vector3(*self.a)
        p_msg.acceleration.angular = geometry_msgs.Vector3(*self.alpha)
        return p_msg

    def from_message(self, msg):
        """Parse a trajectory point message of type `uuv_control_msgs/TrajectoryPoint`
        into the `uuv_trajectory_generator/TrajectoryPoint`.
        
        > *Input arguments*
        
        * `msg` (*type:* `uuv_control_msgs/TrajectoryPoint`): Input trajectory message
        """
        t = msg.header.stamp.to_sec()
        p = msg.pose.position
        q = msg.pose.orientation
        v = msg.velocity.linear
        w = msg.velocity.angular
        a = msg.acceleration.linear
        al = msg.acceleration.angular

        self._t = t
        self._pos = np.array([p.x, p.y, p.z])
        self._rot = np.array([q.x, q.y, q.z, q.w])
        self._vel = np.array([v.x, v.y, v.z, w.x, w.y, w.z])
        self._acc = np.array([a.x, a.y, a.z, al.x, al.y, al.z])
        return True

    def from_dict(self, data):
        """Initialize the trajectory point attributes from a `dict`.
        
        > *Input arguments*
        
        * `data` (*type:* `dict`): Trajectory point as a `dict`
        """
        self._t = data['time']
        self._pos = np.array(data['pos'])
        if self._rot.size == 3:
            self._rot = quaternion_from_euler(data['rot'])
        else:
            self._rot = np.array(data['rot'])
        self._vel = np.array(data['vel'])
        self._acc = np.array(data['acc'])

    def to_dict(self):
        """Convert trajectory point to `dict`.
        
        > *Returns*
        
        Trajectory points data as a `dict`
        """
        data = dict(time=self._t,
                    pos=self._pos,
                    rot=self._rot,
                    vel=self._vel,
                    acc=self._acc)
        return data
