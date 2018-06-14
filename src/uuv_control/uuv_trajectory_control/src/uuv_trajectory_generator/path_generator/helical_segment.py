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

class HelicalSegment(object):
    def __init__(self, center, radius, n_turns, delta_z, angle_offset, is_clockwise=True):
        self._center = np.array(center)
        assert self._center.size == 3, 'Size of center point vector must be 3'

        assert radius > 0, 'Helix radius must be greater than zero'
        assert n_turns > 0, 'Number of turns must be greater than zero'
        assert isinstance(is_clockwise, bool), 'is_clockwise flag must be a boolean'
        
        self._radius = radius
        self._n_turns = n_turns
        self._angle_offset = angle_offset
        self._is_clockwise = is_clockwise
        self._delta_z = delta_z
        self._step_z = float(self._delta_z) / self._n_turns

    def get_length(self):
        return self._n_turns * np.sqrt(self._step_z**2 + (2 * np.pi * self._radius)**2)

    def get_pitch(self):
        return np.sin(self._step_z / np.sqrt(self._step_z**2 + (2 * np.pi * self._radius)**2))

    def interpolate(self, u):
        u = max(u, 0)
        u = min(u, 1)
        delta = 1 if self._is_clockwise else -1        
        x = self._radius * np.cos(self._n_turns * 2 * np.pi * u * delta + self._angle_offset)
        y = self._radius * np.sin(self._n_turns * 2 * np.pi * u * delta + self._angle_offset)
        z = self._n_turns * u * self._step_z

        return self._center + np.array([x, y, z])

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    u = np.linspace(0, 1, 100)

    fig = plt.figure()
    ax = fig.add_subplot(221, projection='3d')

    radius = 3
    center = [2, 2, 2]
    n_turns = 2
    delta_z = 1
    angle_offset = 0.0
    is_clockwise = True

    helix = HelicalSegment(center, radius, n_turns, delta_z, angle_offset, is_clockwise)

    pnts = np.array([helix.interpolate(i) for i in u])
    ax.plot(pnts[:, 0], pnts[:, 1], pnts[:, 2])
    ax.plot([pnts[0, 0]], [pnts[0, 1]], [pnts[0, 2]], '.r')

    print 1, helix.get_pitch() * 180 / np.pi
    ######################################################
    ax = fig.add_subplot(222, projection='3d')

    radius = 3
    center = [2, 2, 2]
    n_turns = 2
    delta_z = -10
    angle_offset = 0.0
    is_clockwise = True

    helix = HelicalSegment(center, radius, n_turns, delta_z, angle_offset, is_clockwise)

    pnts = np.array([helix.interpolate(i) for i in u])
    ax.plot(pnts[:, 0], pnts[:, 1], pnts[:, 2])
    ax.plot([pnts[0, 0]], [pnts[0, 1]], [pnts[0, 2]], '.r')

    print 2, helix.get_pitch() * 180 / np.pi
    ######################################################
    ax = fig.add_subplot(223, projection='3d')

    radius = 3
    center = [2, 2, 2]
    n_turns = 1.2
    delta_z = 1
    angle_offset = 90 * np.pi / 180
    is_clockwise = True

    helix = HelicalSegment(center, radius, n_turns, delta_z, angle_offset, is_clockwise)

    pnts = np.array([helix.interpolate(i) for i in u])
    ax.plot(pnts[:, 0], pnts[:, 1], pnts[:, 2])
    ax.plot([pnts[0, 0]], [pnts[0, 1]], [pnts[0, 2]], '.r')

    print 3, helix.get_pitch() * 180 / np.pi
    ######################################################
    ax = fig.add_subplot(224, projection='3d')

    radius = 3
    center = [2, 2, 2]
    n_turns = 5
    delta_z = 3
    angle_offset = 90 * np.pi / 180
    is_clockwise = False

    helix = HelicalSegment(center, radius, n_turns, delta_z, angle_offset, is_clockwise)

    pnts = np.array([helix.interpolate(i) for i in u])
    ax.plot(pnts[:, 0], pnts[:, 1], pnts[:, 2])
    ax.plot([pnts[0, 0]], [pnts[0, 1]], [pnts[0, 2]], '.r')

    print 4, helix.get_pitch() * 180 / np.pi
    plt.show()
