#!/usr/bin/env python
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

from netCDF4 import Dataset
import numpy as np
from scipy.interpolate import griddata
from time import strptime
from datetime import timedelta, datetime
import datetime
from uuv_nc_parser.srv import *

class NCParser(object):
    MIN2SEC = 60
    HOUR2MIN = 60
    DAY2HOUR = 24

    def __init__(self, filename, labels=None):
        self._data = Dataset(filename, 'r')

        self._labels = dict(x='xc',
                            y='yc',
                            z='zc',
                            time='time')

        if labels is not None:
            assert type(labels) == dict, 'Labels input should be an dict'
            for key in labels:
                if labels[key] not in self._data.variables.keys():
                    print key, ' not a valid NC data variables'
                elif key in self._labels:
                    self._labels[key] = labels[key]
                else:
                    raise self.__class__.__name__, '- Invalid label and key to NC data variable'

        self._time = self._data.variables[self._labels['time']][:]

        if 'minutes' in self._data.variables[self._labels['time']].units:
            self._time *= self.MIN2SEC
            self._time_unit = 'minutes'
        elif 'hours' in self._data.variables[self._labels['time']].units:
            self._time *= self.HOUR2MIN * self.MIN2SEC
            self._time_unit = 'hours'
        elif 'days' in self._data.variables[self._labels['time']].units:
            self._time *= self.DAY2HOUR * self.HOUR2MIN * self.MIN2SEC
            self._time_unit = 'days'
        else:
            self._time_unit = 'seconds'

        print 'Time unit=', self._time_unit

    def __str__(self):
        msg = 'Ocean data, name=%s\n' % self._data.title
        msg += 'Time range [s]=%f to %f\n' % (self.start_time, self.end_time)
        msg += 'Min. X [m]=%f\n' % self.min_x
        msg += 'Max. X [m]=%f\n' % self.max_x
        msg += 'Min. Y [m]=%f\n' % self.min_y
        msg += 'Max. Y [m]=%f\n' % self.max_y
        msg += 'Variables=\n'
        for var_name in self._data.variables:
            msg += '\t%s\n' % var_name
        return msg

    @property
    def variable_names(self):
        return self._data.variables.keys()

    @property
    def start_time(self):
        return self._time[0]

    @property
    def end_time(self):
        return self._time[-1]

    @property
    def time(self):
        return self._time

    @property
    def x(self):
        return self._data.variables[self._labels['x']][:]

    @property
    def len_x(self):
        return self._data.variables[self._labels['x']].shape[0]

    @property
    def min_x(self):
        return self._data.variables[self._labels['x']][0]

    @property
    def max_x(self):
        return self._data.variables[self._labels['x']][-1]

    @property
    def y(self):
        return self._data.variables[self._labels['y']][:]

    @property
    def len_y(self):
        return self._data.variables[self._labels['y']].shape[0]

    @property
    def min_y(self):
        return self._data.variables[self._labels['y']][0]

    @property
    def max_y(self):
        return self._data.variables[self._labels['y']][-1]

    @property
    def z(self):
        return self._data.variables[self._labels['z']][:]

    @property
    def len_z(self):
        return self._data.variables[self._labels['z']].shape[0]

    @property
    def min_z(self):
        return self._data.variables[self._labels['z']][0]

    @property
    def max_z(self):
        return self._data.variables[self._labels['z']][-1]

    def print_variable(self, var_name):
        if var_name not in self.variable_names:
            print 'Invalid variable name, var_name=', var_name
            return
        print self._data.variables[var_name]

    def get_closest_xy_idx(self, x, y):
        idx = np.argmin(np.abs(self.x - x))
        idy = np.argmin(np.abs(self.y - y))
        return idx, idy

    def get_range(self, value, var_name):
        if var_name not in self._labels.keys():
            print 'Invalid coordinate var_name'
            return None, None

        label = self._labels[var_name]

        if var_name in ['x', 'y', 'z']:
            vec = self._data.variables[label][:]
        elif var_name == 'time':
            vec = self.time
        else:
            vec = self._data.variables[label][:]

        i = np.argmin(np.abs(vec - value))

        if i == 0:
            return i, i + 1
        elif i == vec.shape[0] - 1:
            return i - 1, i
        elif vec[i] >= value:
            return i - 1, i
        else:
            return i, i + 1

    def print_box(self, x, y, depth):
        idx = self.get_range(x, 'x')
        idy = self.get_range(y, 'y')
        idz = self.get_range(depth, 'z')

        print 'x, y, z=', x, y, depth
        print 'box(x)=', self.x[idx[0]], self.x[idx[1]]
        print 'box(y)=', self.y[idy[0]], self.y[idy[1]]
        print 'box(z)=', self.z[idz[0]], self.z[idz[1]]

    def interpolate(self, variable, x, y, z, time):
        if variable not in self.variable_names:
            print 'Invalid variable name, value=', variable
            return None

        idx = self.get_range(x, 'x')
        idy = self.get_range(y, 'y')
        idz = self.get_range(z, 'z')
        idt = self.get_range(time, 'time')

        var = self._data.variables[variable]

        values = None
        pnts = None

        if len(var.shape) == 2:
            sample_coord = (y, x)
            for i in range(2):
                for j in range(2):
                    coord = np.array([self.y[idy[i]], self.x[idx[j]]])
                    v = np.ma.getdata(var[idy[i]:idy[i] + 1,
                                          idx[j]:idx[j] + 1]).flatten()[0]
                    if pnts is None:
                        pnts = coord
                    else:
                        pnts = np.vstack((pnts, coord))

                    if values is None:
                        values = np.array([v])
                    else:
                        values = np.vstack((values, v))
        elif len(var.shape) == 3:
            sample_coord = (time, y, x)
            for i in range(2):
                for j in range(2):
                    for k in range(2):
                        coord = np.array([self.time[idt[i]],
                                          self.y[idy[j]],
                                          self.x[idx[k]]])
                        v = np.ma.getdata(var[idt[i]:idt[i] + 1,
                                              idy[j]:idy[j] + 1,
                                              idx[k]:idx[k] + 1]).flatten()[0]
                        if pnts is None:
                            pnts = coord
                        else:
                            pnts = np.vstack((pnts, coord))

                        if values is None:
                            values = np.array([v])
                        else:
                            values = np.vstack((values, v))
        elif len(var.shape) == 4:
            sample_coord = (time, z, y, x)
            for i in range(2):
                for j in range(2):
                    for k in range(2):
                        for u in range(2):
                            coord = np.array([self.time[idt[i]],
                                              self.z[idz[j]],
                                              self.y[idy[k]],
                                              self.x[idx[u]]])
                            v = np.ma.getdata(var[idt[i]:idt[i] + 1,
                                                  idz[j]:idz[j] + 1,
                                                  idy[k]:idy[k] + 1,
                                                  idx[u]:idx[u] + 1]).flatten()[0]
                            if pnts is None:
                                pnts = coord
                            else:
                                pnts = np.vstack((pnts, coord))

                            if values is None:
                                values = np.array([v])
                            else:
                                values = np.vstack((values, v))
        else:
            return None

        # Filling for NaN values with the minimum
        if values[values == 0].size > 0 and values[values != 0].size > 0:
            values[values == 0] = values[values != 0].min()

        uniq = np.unique(values)
        if uniq.size == 1:
            return uniq[0]

        try:
            output = griddata(pnts, values, sample_coord, method='linear')
            return output[0]
        except:
            return 0.0
