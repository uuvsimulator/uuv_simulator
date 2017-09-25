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


class MaxError(KPI):
    TAG = 'max_error'
    LABEL = 'Max. Error'
    UNIT = 'm'
    TARGET = 'error'

    def __init__(self, error_elem='position', use_bag=True, time_offset=0.0):
        KPI.__init__(self, use_bag, time_offset)
        self._kpi_arg = error_elem

        if self._error_set is not None:
            assert error_elem in self._error_set.get_tags(), 'Error element given does not exist'
            # Initialize the data structure for this KPI
            self._input_values = dict(error=self._error_set.get_data(error_elem, time_offset))
        else:
            self._input_values = None

    def compute(self, input_values=None):
        assert input_values is not None or self._input_values is not None, 'No input data to process'
        if self._input_values is None:
            assert self.is_iterable(input_values), 'Invalid input data'
            self._input_values = dict(error=input_values)

        self._kpi_value = self.get_max_error(self._input_values['error'])
        return self._kpi_value
