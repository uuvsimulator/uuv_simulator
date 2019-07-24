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
import numpy
from thruster import Thruster


class ThrusterProportional(Thruster):
    """This model corresponds to the linear relation between a function
    abs(command)*command of the command input (usually the command angular
    velocity) to the thrust force. A constant gain has to be provided.


    > *Input arguments*
    
    * `index` (*type:* `int`): Thruster's ID.
    * `topic` (*type:* `str`): Thruster's command topic.
    * `pos` (*type:* `numpy.array` or `list`): Position vector 
    of the thruster with respect to the vehicle's frame.
    * `orientation` (*type:* `numpy.array` or `list`): Quaternion 
    with the orientation of the thruster with respect to the vehicle's
    frame as `(qx, qy, qz, qw)`.
    * `axis` (*type:* `numpy.array`): Axis of rotation of the thruster.
    * `gain` (*type:* `float`): Constant gain.
    """
    LABEL = 'proportional'

    def __init__(self, *args, **kwargs):
        super(ThrusterProportional, self).__init__(*args)

        if 'gain' not in kwargs:
            rospy.ROSException('Thruster gain not given')
        self._gain = kwargs['gain']
        rospy.loginfo('Thruster model:')
        rospy.loginfo('\tGain=' + str(self._gain))

    def get_command_value(self, thrust):
        """Compute the angular velocity necessary 
        for the desired thrust force.
        
        > *Input arguments*
        
        * `thrust` (*type:* `float`): Thrust force magnitude in N
        
        > *Returns*
        
        `float`: Angular velocity set-point for the thruster in rad/s 
        """
        return numpy.sign(thrust) * \
            numpy.sqrt(numpy.abs(thrust) / self._gain)

    def get_thrust_value(self, command):
        """Computes the thrust force for the given angular velocity
        set-point.
        
        > *Input arguments*
        
        * `command` (*type:* `float`): Angular velocity set-point for 
        the thruster in rad/s 
        
        > *Returns*

        `thrust` (*type:* `float`): Thrust force magnitude in N
        """
        return self._gain * numpy.abs(command) * command
