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
from bag_evaluation import Recording, ErrorSet


class KPI(object):
    TAG = ''
    LABEL = ''
    UNIT = ''
    TARGET = ''

    def __init__(self, use_bag=True, time_offset=0.0):
        assert time_offset >= 0.0, 'Time offset cannot be negative'
        self._input_values = dict()

        self._bag = None
        self._error_set = None
        self._time_offset = time_offset

        if use_bag:
            self._bag = Recording.get_instance()
            assert self._bag is not None, 'No recording found, data has not been parsed'
            self._error_set = ErrorSet.get_instance()
            assert self._error_set is not None, 'Error set has not been initialized'

        self._kpi_value = None
        self._kpi_arg = ''

    def __str__(self):
        msg = 'KPI Information\n'
        msg += 'tag=' + str(self.TAG) + '\n'
        msg += 'label=' + str(self.LABEL) + '\n'
        msg += 'unit=' + str(self.UNIT) + '\n'
        if len(self._kpi_arg):
            msg += 'arg=' + self._kpi_arg + '\n'
        return msg

    @property
    def full_tag(self):
        output = dict()
        key = self.TAG
        if len(self._kpi_arg):
            key += '_' + self._kpi_arg
        return key

    @property
    def is_init(self):
        return len(self._input_values.keys()) > 0

    @property
    def kpi_value(self):
        return self._kpi_value

    @property
    def target(self):
        return self.TARGET

    @property
    def unit(self):
        return self.UNIT

    @property
    def label(self):
        return self.LABEL

    @property
    def tag(self):
        return self.TAG

    @property
    def value(self):
        return self._input_values

    @staticmethod
    def get_all_kpis():
        return KPI.__subclasses__()

    @staticmethod
    def get_kpi(tag, *args):
        assert tag in KPI.get_all_kpi_tags(), 'Invalid KPI tag, value=' + str(tag)
        for kpi in KPI.get_all_kpis():
            if kpi.TAG == tag:
                return kpi(*args)
        return None

    @staticmethod
    def get_kpi_target(tag):
        assert tag in KPI.get_all_kpi_tags(), 'Invalid KPI tag, value=' + str(tag)
        for kpi in KPI.get_all_kpis():
            if kpi.TAG == tag:
                return kpi.TARGET
        return None

    @staticmethod
    def get_all_kpi_tags():
        return [kpi.TAG for kpi in KPI.get_all_kpis()]

    @staticmethod
    def get_all_kpi_labels():
        return [kpi.LABEL for kpi in KPI.get_all_kpis()]

    @staticmethod
    def get_n_kpis():
        return len(KPI.__subclasses__())

    @staticmethod
    def get_squared(values):
        if type(values[0]) not in [float, np.float64]:
            output = [e.dot(e) for e in values]
        else:
            output = [e**2 for e in values]
        return output

    @staticmethod
    def get_rms_error(values):
        n = len(values)
        if type(values[0]) not in [float, np.float64]:
            output = np.sqrt(np.sum(KPI.get_squared(values), axis=0) / n)
        else:
            output = np.sqrt(np.sum(KPI.get_squared(values)) / n)
        return output

    @staticmethod
    def get_error(values):
        dist = np.sqrt(KPI.get_squared(values))
        return dist

    @staticmethod
    def get_max_error(values):
        return KPI.get_error(values).max()

    @staticmethod
    def get_mean_error(values):
        return KPI.get_error(values).mean()

    def is_iterable(self, input_values):
        try:
            it = iter(input_values)
        except TypeError:
            print 'Input values are not iterable'
            return False
        return True

    def set_input_values(self, values):
        for tag in self._input_values:
            if tag not in values:
                print 'Invalid input values set'
                return False
        self._input_values = values
        return True

    def compute(self):
        raise NotImplementedError()
