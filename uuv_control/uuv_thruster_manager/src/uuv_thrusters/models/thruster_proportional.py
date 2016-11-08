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
from thruster import Thruster


class ThrusterProportional(Thruster):
    """This model corresponds to the linear relation between a function
    abs(command)*command of the command input (usually the command angular
    velocity) to the thrust force. A constant gain has to be provided.
    """
    LABEL = 'proportional'

    def __init__(self, *args, **kwargs):
        super(ThrusterProportional, self).__init__(*args)

        if 'gain' not in kwargs:
            rospy.ROSException('Thruster gain not given')
        self._gain = kwargs['gain']

    def get_command_value(self, thrust):
        return numpy.sign(self._thrust) * \
            numpy.sqrt(numpy.abs(self._thrust) / self._gain)

    def get_thrust_value(self, command):
        """Computes the thrust force for the given command."""
        return self._gain * numpy.abs(command) * command
