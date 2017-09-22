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


    def __init__(self, use_bag=True, time_offset=0.0):
        KPI.__init__(self, use_bag, time_offset)

        if self._bag is not None:
            # Initialize the data structure for this KPI
            self._input_values = dict()
            self._t = None
            for i in range(self._bag.n_thrusters):
                t, thrusts = self._bag.get_thruster_data(i)
                if self._t is None:
                    self._t = np.array(t)
                    assert time_offset < np.max(self._t), 'Time offset out of range'                
                self._input_values[i] = np.array(thrusts)
                self._input_values[i] = self._input_values[i][np.nonzero(self._t >= self._time_offset)[0]]
        else:
            self._input_values = None

    def compute(self, input_values=None):
        assert input_values is not None or self._input_values is not None, 'No input data to process'

        if self._input_values is None:
            assert type(input_values) is dict, 'Input dict is not a dictionary'
            assert len(input_values.keys()) > 0, 'Dictionary is empty'
            self._input_values = dict()
            for i, tag in enumerate(input_values.keys()):
                assert i == tag, 'Thruster indexes must be the keys of the dictionary'
                assert self.is_iterable(input_values[tag]), 'No valid thrust force data'
                self._input_values[tag] = np.array(input_values)

        self._kpi_value = np.max([np.max(np.abs(self._input_values[i])) for i in self._input_values])
        return self._kpi_value
