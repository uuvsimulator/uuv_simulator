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
from scipy.misc import factorial


class BezierCurve(object):
    """
    Implementation of Bezier curves of orders 3, 4 and 5 based on [1].
    
    [1] Biagiotti, Luigi, and Claudio Melchiorri. Trajectory planning for 
        automatic machines and robots. Springer Science & Business Media, 2008.
    """

    def __init__(self, pnts, order, tangents=None):
        assert order in [3, 4, 5], 'Invalid Bezier curve order'
        assert type(pnts) == list and len(pnts) >= 2, 'At least two points are needed to calculate the curve'

        self._pnts = list()
        for pnt in pnts:
            if type(pnt) == list:
                assert len(pnt) == 3, 'Point must have three elements'
                self._pnts.append(np.array(pnt))
            elif type(pnt) == np.ndarray:
                assert pnt.size == 3, 'Point must have three elements'
                self._pnts.append(pnt)
            else:
                raise TypeError('Point in list is neither a list or an array')

        if tangents is not None:
            assert type(tangents) == list and len(tangents) == 2, 'Tangent vectors must be provided'
            for t in tangents:
                if type(t) == list:
                    assert len(t) == 3, 'Tangent vector must have three elements'
                elif type(t) == np.ndarray:
                    assert t.size == 3, 'Tangent vector must have three elements'
                else:
                    raise TypeError('Tangent vector is neither a list or an array')

        self._control_pnts = [np.zeros(3) for _ in range(order + 1)]

        self._order = order
        if self._order == 3:
            assert len(self._pnts) == 2, 'Two points are needed for the curve to be computed'
            # Setting initial and last control points
            self._control_pnts[0] = self._pnts[0]
            self._control_pnts[3] = self._pnts[1]
            # Compute alpha
            a = 16 - np.linalg.norm(tangents[0] + tangents[1])**2
            b = 12 * np.dot(self._control_pnts[3] - self._control_pnts[0], tangents[0] + tangents[1])
            c = -36 * np.linalg.norm(self._control_pnts[3] - self._control_pnts[0])**2
            alpha = np.roots([a, b, c]).max()

            # Compute the rest of the control points
            self._control_pnts[1] = self._control_pnts[0] + (1.0 / 3) * alpha * tangents[0]
            self._control_pnts[2] = self._control_pnts[3] - (1.0 / 3) * alpha * tangents[1]
        elif self._order == 4:
            assert len(self._pnts) == 3, 'Three points are needed for the curve to be computed'
            # Setting initial control points
            self._control_pnts[0] = self._pnts[0]
            self._control_pnts[2] = self._pnts[1]
            self._control_pnts[4] = self._pnts[2]

            radius = np.linalg.norm(self._pnts[0] - self._pnts[1])
            tangents = list()
            tangents.append((self._pnts[1] - self._pnts[0]) / radius)
            tangents.append((self._pnts[2] - self._pnts[1]) / radius)

            # Compute alpha
            a = 4 - (1.0 / 4) * np.linalg.norm(tangents[0] + tangents[1])**2
            b = 3 * np.dot(self._control_pnts[4] - self._control_pnts[0], tangents[0] + tangents[1])
            c = -9 * np.linalg.norm(self._control_pnts[4] - self._control_pnts[0])**2
            alpha = np.roots([a, b, c]).max()

            # Compute the rest of the control points
            self._control_pnts[1] = self._control_pnts[0] + 0.25 * alpha * tangents[0]
            self._control_pnts[3] = self._control_pnts[4] - 0.25 * alpha * tangents[1]
        elif self._order == 5:
            assert len(self._pnts) == 3, 'Three points are needed for the curve to be computed'
            # Setting initial control points
            self._control_pnts[0] = self._pnts[0]
            self._control_pnts[5] = self._pnts[2]

            radius = np.linalg.norm(self._pnts[0] - self._pnts[1])
            tangents = list()
            tangents.append((self._pnts[1] - self._pnts[0]) / radius)
            tangents.append((self._pnts[2] - self._pnts[1]) / radius)

            # Compute alpha
            a = 256 - 49 * np.linalg.norm(tangents[0] + tangents[1])**2
            b = 420 * np.dot(self._control_pnts[5] - self._control_pnts[0], tangents[0] + tangents[1])
            c = -900 * np.linalg.norm(self._control_pnts[5] - self._control_pnts[0])**2
            alpha = np.roots([a, b, c]).max()

            # Compute the rest of the control points
            self._control_pnts[1] = self._control_pnts[0] + 0.2 * alpha * tangents[0]
            self._control_pnts[2] = 2 * self._control_pnts[1] - self._control_pnts[0]
            self._control_pnts[4] = self._control_pnts[5] - 0.2 * alpha * tangents[1]
            self._control_pnts[3] = 2 * self._control_pnts[4] - self._control_pnts[5]

    def control_pnts(self):
        return self._control_pnts

    def interpolate(self, u):
        u = max(u, 0)
        u = min(u, 1)

        b = np.zeros(3)
        n = len(self._control_pnts) - 1
        for i in range(len(self._control_pnts)):
            b = b + self._get_binomial(n, i) * (1 - u)**(n - i) * u**i * self._control_pnts[i]
        return b

    def get_length(self):
        return self._order * np.linalg.norm(self._control_pnts[1] - self._control_pnts[0])


    @staticmethod
    def _get_binomial(n, i):
        return factorial(n) / (factorial(i) * factorial(n - i))

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    print 'Test - Cubic Bezier curve'
    q_x = [0, 1, 2, 4, 5, 6]
    q_y = [0, 2, 3, 3, 2, 0]
    q_z = [0, 1, 0, 0, 2, 2]

    q = np.vstack((q_x, q_y, q_z)).T

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(q[:, 0], q[:, 1], q[:, 2], 'b--')
    ax.plot(q[:, 0], q[:, 1], q[:, 2], 'ro')

    # Compute the distance between points
    lengths = [np.linalg.norm(q[i + 1, :] - q[i, :]) for i in range(q.shape[0] - 1)]
    lengths = [0] + lengths
    u = np.cumsum(lengths) / np.sum(lengths)
    print q.T
    print 'u=', u

    tangents = [np.zeros(3) for _ in range(q.shape[0])]
    delta_u = lambda k: u[k] - u[k - 1]
    delta_q = lambda k: q[k, :] - q[k - 1, :]
    lamb_k = lambda k: delta_q(k) / delta_u(k)
    alpha_k = lambda k: delta_u(k) / (delta_u(k) + delta_u(k + 1))
    for i in range(1, len(u) - 1):
        tangents[i] = (1 - alpha_k(i)) * lamb_k(i) + alpha_k(i) * lamb_k(i + 1)
        if i == 1:
            tangents[0] = 2 * lamb_k(i) - tangents[1]

    tangents[-1] = 2 * lamb_k(len(u) - 1) - tangents[-2]

    print 'Tangents'
    for i in range(len(tangents)):
        tangents[i] = tangents[i] / np.linalg.norm(tangents[i])
        print '\t#%d - %.2f %.2f %.2f' % (i, tangents[i][0], tangents[i][1], tangents[i][2])
        ax.plot([q[i, 0], tangents[i][0] + q[i, 0]],
                [q[i, 1], tangents[i][1] + q[i, 1]],
                [q[i, 2], tangents[i][2] + q[i, 2]], 'c')

    segments = list()
    print 'Segments'
    for i in range(len(tangents) - 1):
        segments.append(BezierCurve([q[i, :], q[i + 1, :]], 3, tangents[i:i + 2]))
        print '\t', segments[-1]._control_pnts

    lengths = [seg.get_length() for seg in segments]
    lengths = [0] + lengths
    total_length = np.sum(lengths)

    u = np.cumsum(lengths) / total_length

    pnts = None
    for i in np.linspace(0, 1, 100):
        idx = (u - i >= 0).nonzero()[0][0]
        if idx == 0:
            u_k = 0
            pnts = segments[idx].interpolate(u_k)
        else:
            u_k = (i - u[idx - 1]) / (u[idx] - u[idx - 1])
            pnts = np.vstack((pnts, segments[idx - 1].interpolate(u_k)))

    ax.plot(pnts[:, 0], pnts[:, 1], pnts[:, 2], 'g')

    fig = plt.figure()

    u_hat = np.linspace(0, 1, 100)
    d_x = [(pnts[i, 0] - pnts[i - 1, 0]) / (u_hat[i] - u_hat[i - 1]) for i in range(1, pnts.shape[0])]
    d_y = [(pnts[i, 1] - pnts[i - 1, 1]) / (u_hat[i] - u_hat[i - 1]) for i in range(1, pnts.shape[0])]
    d_z = [(pnts[i, 2] - pnts[i - 1, 2]) / (u_hat[i] - u_hat[i - 1]) for i in range(1, pnts.shape[0])]

    ax = fig.add_subplot(311)
    ax.plot(u_hat[1::], d_x)
    ax.set_xlim([u_hat[1::].min(), u_hat[1::].max()])
    ax.grid(True)

    ax = fig.add_subplot(312)
    ax.plot(u_hat[1::], d_y)
    ax.set_xlim([u_hat[1::].min(), u_hat[1::].max()])
    ax.grid(True)

    ax = fig.add_subplot(313)
    ax.plot(u_hat[1::], d_z)
    ax.set_xlim([u_hat[1::].min(), u_hat[1::].max()])
    ax.grid(True)

    plt.show()

