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

import numpy


class PIDRegulator:
    """A very basic 1D PID Regulator."""
    def __init__(self, p, i, d, sat):
        self.p = p
        self.i = i
        self.d = d
        self.sat = sat

        self.integral = 0
        self.prev_err = 0
        self.prev_t = -1.0

    def __str__(self):
        msg = 'PID controller:'
        msg += '\n\tp=%f' % self.p
        msg += '\n\ti=%f' % self.i
        msg += '\n\td=%f' % self.d
        msg += '\n\tsat=%f' % self.sat
        return msg

    def regulate(self, err, t):
        derr_dt = 0.0
        dt = t - self.prev_t
        if self.prev_t > 0.0 and dt > 0.0:
            derr_dt = (err - self.prev_err)/dt
            self.integral += 0.5*(err + self.prev_err)*dt

        u = self.p*err + self.d*derr_dt + self.i*self.integral

        self.prev_err = err
        self.prev_t = t

        if (numpy.linalg.norm(u) > self.sat):
            # controller is in saturation: limit outpt, reset integral
            u = self.sat*u/numpy.linalg.norm(u)
            self.integral = 0.0

        return u
