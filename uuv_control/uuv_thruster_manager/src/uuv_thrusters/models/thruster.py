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
import numpy
import tf.transformations as trans
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header


class Thruster(object):
    """Abstract function to all the thruster models avaialble. The instance
    of a thruster model must use the factory method."""

    LABEL = ''

    def __init__(self, index, topic, pos, orientation):
        """Thruster class constructor.

        Parameters
        ----------
        index: int
            Thruster's index.
        topic: str
            Name of the thruster's command topic.
        pos: numpy.array
            3D position of the thruster with relation to the reference frame.
        orientation: numpy.array
            Quaternion with the orientation of the thruster with relation to
            the reference frame.
        """
        self._index = index
        self._topic = topic
        self._pos = None
        self._orientation = None
        self._force_dist = None
        if None not in [pos, orientation]:
            self._pos = pos
            self._orientation = orientation
            # compute contribution to configuration matrix of this thruster
            thrust_body = trans.quaternion_matrix(orientation).dot(
                numpy.array([1, 0, 0, 0]).transpose())[0:3]
            torque_body = numpy.cross(pos, thrust_body)
            self._force_dist = numpy.hstack((
                thrust_body, torque_body)).transpose()
        self._command = 0
        self._thrust = 0
        self._command_pub = rospy.Publisher(self._topic, FloatStamped,
                                            queue_size=10)

        print 'Thruster #%d - %s - %s' % (self._index, self.LABEL, self._topic)

    @property
    def tam_column(self):
        return self._force_dist

    @staticmethod
    def create_thruster(model_name, *args, **kwargs):
        for thruster in Thruster.__subclasses__():
            if model_name == thruster.LABEL:
                return thruster(*args, **kwargs)
        rospy.ROSException('Invalid thruster model')

    def get_command_value(self, thrust):
        """Convert desired thrust force to input command according to this
        function. Overwrite this method to implement custom models."""
        return 0

    def get_thrust_value(self, command):
        """Computes the thrust force for the given command."""
        return 0

    def get_curve(self, min_value, max_value, n_points):
        """Sample the conversion curve and return the values."""
        if min_value >= max_value or n_points <= 0:
            return [], []
        input_values = numpy.linspace(min_value, max_value, n_points)
        output_values = []
        for value in input_values:
            output_values.append(self.get_thrust_value(value))
        return input_values.tolist(), output_values

    def _calc_command(self):
        """Convert the desired thrust force into angular velocity command
        according using a gain."""
        self._command = self.get_command_value(self._thrust)

    def _update(self, thrust):
        self._thrust = thrust
        self._calc_command()

    def publish_command(self, thrust):
        self._update(thrust)
        output = FloatStamped()
        output.header.stamp = rospy.Time.now()
        output.data = self._command
        self._command_pub.publish(output)
