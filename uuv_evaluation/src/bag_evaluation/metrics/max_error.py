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

from .kpi import KPI


class MaxError(KPI):
    TAG = 'max_error'
    LABEL = 'Max. Error'
    UNIT = 'm'
    TARGET = 'error'

    def __init__(self, error_elem='position'):
        KPI.__init__(self)
        assert error_elem in self._error_set.get_tags(), 'Error element given does not exist'
        # Initialize the data structure for this KPI
        self._kpi_arg = error_elem
        self._input_values = dict(error=self._error_set.get_data(error_elem))

    def compute(self):
        self._kpi_value = self.get_max_error(self._input_values['error'])
        return self._kpi_value
