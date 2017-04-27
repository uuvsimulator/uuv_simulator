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

import numpy as np
from .kpi import KPI


class MaxAbsThrust(KPI):
    TAG = 'max_abs_thrust'
    LABEL = 'Maximum absolute thrust'
    UNIT = 'N'
    TARGET = 'thruster'


    def __init__(self):
        KPI.__init__(self)
        # Initialize the data structure for this KPI
        self._input_values = dict()
        for i in range(self._bag.n_thrusters):
            t, thrusts = self._bag.get_thruster_data(i)
            self._input_values[i] = thrusts

    def compute(self):
        self._kpi_value = np.max([np.max(np.abs(self._input_values[i])) for i in self._input_values])
        return self._kpi_value
