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

    def __init__(self, pnts, order, tangents=None, normals=None):
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
            if len(self._pnts) == 3:            
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
            elif len(self._pnts) == 2:                
                assert tangents is not None and normals is not None
                assert isinstance(tangents, list) and len(tangents) == 2, 'Tangent vectors must be provided'
                assert isinstance(normals, list) and len(normals) == 2, 'Normal vectors must be provided'

                beta_hat = 0.51
                                
                a = beta_hat**2 * np.linalg.norm(normals[1] - normals[0])**2
                b = -28 * beta_hat * np.dot((tangents[0] + tangents[1]), normals[1] - normals[0])
                c = 196 * np.linalg.norm(tangents[0] + tangents[1])**2 + 120 * beta_hat * np.dot(self._pnts[1] - self._pnts[0], normals[1] - normals[0]) - 1024
                d = -1680 * np.dot(self._pnts[1] - self._pnts[0], tangents[0] + tangents[1])
                e = 3600 * np.linalg.norm(self._pnts[1] - self._pnts[0])**2
                    
                alpha_k = np.real(np.roots([a, b, c, d, e])).max()

                # Setting initial control points
                self._control_pnts[0] = self._pnts[0]
                self._control_pnts[5] = self._pnts[1]

                self._control_pnts[1] = self._control_pnts[0] + alpha_k / 5.0 * tangents[0]
                self._control_pnts[2] = 2.0 * self._control_pnts[1] - self._control_pnts[0] + beta_hat * alpha_k**2 / 20.0 * normals[0]
                self._control_pnts[4] = self._control_pnts[5] - alpha_k / 5.0 * tangents[1]
                self._control_pnts[3] = 2.0 * self._control_pnts[4] - self._control_pnts[5] + beta_hat * alpha_k**2 / 20.0 * normals[1]

    @staticmethod
    def distance(p1, p2):
        p1 = np.array(p1)
        p2 = np.array(p2)

        assert p1.size == 3 and p2.size == 3        
        return np.sqrt(np.sum((p2 - p1)**2))

    @staticmethod
    def generate_cubic_curve(pnts):
        assert isinstance(pnts, list), 'List of points is invalid'
        tangents = [np.zeros(3) for _ in range(len(pnts))]

        lengths = [BezierCurve.distance(pnts[i + 1], pnts[i]) for i in range(len(pnts) - 1)]
        lengths = [0] + lengths
        
        # Initial vector of parametric variables for the curve
        u = [l / np.sum(lengths) for l in np.cumsum(lengths)]
        delta_u = lambda k: u[k] - u[k - 1]
        delta_q = lambda k: pnts[k] - pnts[k - 1]
        lamb_k = lambda k: delta_q(k) / delta_u(k)
        alpha_k = lambda k: delta_u(k) / (delta_u(k) + delta_u(k + 1))

        for i in range(1, len(u) - 1):            
            tangents[i] = (1 - alpha_k(i)) * lamb_k(i) + alpha_k(i) * lamb_k(i + 1)
            if i == 1:
                tangents[0] = 2 * lamb_k(i) - tangents[1]

        tangents[-1] = 2 * lamb_k(len(u) - 1) - tangents[-2]

        # Normalize tangent vectors
        for i in range(len(tangents)):
            tangents[i] = tangents[i] / np.linalg.norm(tangents[i])

        segments = list()
        # Generate the cubic Bezier curve segments
        for i in range(len(tangents) - 1):
            segments.append(BezierCurve([pnts[i], pnts[i + 1]], 3, [tangents[i], tangents[i + 1]]))

        return segments

    @staticmethod
    def generate_quintic_curve(pnts):
        assert isinstance(pnts, list), 'List of points is invalid'
        tangents = [np.zeros(3) for _ in range(len(pnts))]
        normals = [np.zeros(3) for _ in range(len(pnts))]

        lengths = [BezierCurve.distance(pnts[i + 1], pnts[i]) for i in range(len(pnts) - 1)]
        lengths = [0] + lengths
        # Initial vector of parametric variables for the curve
        u = np.cumsum(lengths) / np.sum(lengths)                
        
        delta_u = lambda k: u[k] - u[k - 1]
        delta_q = lambda k: pnts[k] - pnts[k - 1]
        lamb_k = lambda k: delta_q(k) / delta_u(k)
        alpha_k = lambda k: delta_u(k) / (delta_u(k) + delta_u(k + 1))
        normal_k = lambda k: ( ((pnts[k + 1] - pnts[k]) / (u[k + 1] - u[k])) - ((pnts[k] - pnts[k - 1]) / (u[k] - u[k - 1])) ) / (u[k + 1] - u[k - 1])

        for i in range(1, len(u) - 1):
            tangents[i] = (1 - alpha_k(i)) * lamb_k(i) + alpha_k(i) * lamb_k(i + 1)
            normals[i] = normal_k(i)
            if i == 1:
                tangents[0] = 2 * lamb_k(i) - tangents[1]
                normals[0] = normal_k(i)

        tangents[-1] = 2 * lamb_k(len(u) - 1) - tangents[-2]
        normals[-1] = normal_k(len(u) - 3)
                
        # Normalize tangent vectors
        for i in range(len(tangents)):
            tangents[i] /= np.linalg.norm(tangents[i])
            normals[i] /= np.linalg.norm(normals[i])

        segments = list()
        # Generate the cubic Bezier curve segments
        for i in range(len(tangents) - 1):
            segments.append(BezierCurve([pnts[i], pnts[i + 1]], 5, 
                [tangents[i], tangents[i + 1]], 
                [normals[i], normals[i + 1]]))

        return segments

    def control_pnts(self):
        return self._control_pnts

    def interpolate(self, u):
        u = max(u, 0)
        u = min(u, 1)

        b = np.zeros(3)        
        for i in range(len(self._control_pnts)):
            b += self.compute_polynomial(self._order, i, u) * self._control_pnts[i]
        return b

    def get_derivative(self, u, order=1):
        u = max(u, 0)
        u = min(u, 1)

        b = np.zeros(3)        
        for i in range(len(self._control_pnts) - order):
            b = b + self._order * self.compute_polynomial(self._order - order, i, u) * \
                 (self._control_pnts[i + 1] - self._control_pnts[i])        
        return b

    def get_length(self):
        return self._order * np.linalg.norm(self._control_pnts[1] - self._control_pnts[0])

    def compute_polynomial(self, n, i, u):
        return self._get_binomial(n, i) * (1 - u)**(n - i) * u**i

    @staticmethod
    def _get_binomial(n, i):
        return factorial(n) / (factorial(i) * factorial(n - i))

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    np.set_printoptions(suppress=True, precision=2)

    print 'Test - Cubic Bezier curve'
    q_x = [0, 1, 2, 4, 5, 6]
    q_y = [0, 2, 3, 3, 2, 0]
    q_z = [0, 1, 0, 0, 2, 2]

    q = [np.array([x, y, z]) for x, y, z in zip(q_x, q_y, q_z)]
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(q_x, q_y, q_z, 'b--')
    ax.plot(q_x, q_y, q_z, 'ro')

    # Compute cubic Bezier curve

    segments, tangents = BezierCurve.generate_cubic_curve(q)

    for i in range(len(q)):
        t = 0.8 * tangents[i]
        ax.plot([q[i][0], q[i][0] + t[0]], [q[i][1], q[i][1] + t[1]], [q[i][2], q[i][2] + t[2]], 'b', linewidth=3)

    lengths = [seg.get_length() for seg in segments]
    lengths = [0] + lengths
    total_length = np.sum(lengths)

    u = np.cumsum(lengths) / total_length

    pnts = None
    deriv = None
    for i in np.linspace(0, 1, 200):
        idx = (u - i >= 0).nonzero()[0][0]
        if idx == 0:
            u_k = 0
            pnts = segments[idx].interpolate(u_k)
            deriv = segments[idx].get_derivative(u_k)
        else:
            u_k = (i - u[idx - 1]) / (u[idx] - u[idx - 1])
            pnts = np.vstack((pnts, segments[idx - 1].interpolate(u_k)))
            deriv = np.vstack((deriv, segments[idx - 1].get_derivative(u_k)))

    ax.plot(pnts[:, 0], pnts[:, 1], pnts[:, 2], 'g')    

    for d, p in zip(deriv, pnts):
        d /= np.linalg.norm(d) 
        pd = p + d 
        # ax.plot([p[0], pd[0]], [p[1], pd[1]], [p[2], pd[2]], 'r')

    ax.set_aspect('equal')

    u = np.array([0, 0.22, 0.38, 0.56, 0.79, 1])

    for i in u:
        idx = (u - i >= 0).nonzero()[0][0]
        if idx == 0:
            u_k = 0            
            deriv = segments[idx].get_derivative(u_k)
        else:
            u_k = (i - u[idx - 1]) / (u[idx] - u[idx - 1])            
            deriv = np.vstack((deriv, segments[idx - 1].get_derivative(u_k)))
        
    # Compute Quintic Bezier curve    
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # ax.plot(q_x, q_y, q_z, 'b--')
    # ax.plot(q_x, q_y, q_z, 'ro')

    # segments, tangents, normals = BezierCurve.generate_quintic_curve(q)

    # for i in range(len(q)):
    #     t = tangents[i]
    #     ax.plot([q[i][0], q[i][0] + t[0]], [q[i][1], q[i][1] + t[1]], [q[i][2], q[i][2] + t[2]], 'b', linewidth=3)

    #     n = normals[i]
    #     ax.plot([q[i][0], q[i][0] + n[0]], [q[i][1], q[i][1] + n[1]], [q[i][2], q[i][2] + n[2]], 'r', linewidth=3)

    # lengths = [seg.get_length() for seg in segments]
    # lengths = [0] + lengths
    # total_length = np.sum(lengths)

    # u = np.cumsum(lengths) / total_length

    # pnts = None
    # deriv = None
    # for i in np.linspace(0, 1, 100):
    #     idx = (u - i >= 0).nonzero()[0][0]
    #     if idx == 0:
    #         u_k = 0
    #         pnts = segments[idx].interpolate(u_k)
    #         deriv = segments[idx].get_derivative(u_k)
    #     else:
    #         u_k = (i - u[idx - 1]) / (u[idx] - u[idx - 1])
    #         pnts = np.vstack((pnts, segments[idx - 1].interpolate(u_k)))
    #         deriv = np.vstack((deriv, segments[idx - 1].get_derivative(u_k)))        

    # for d, p in zip(deriv, pnts):
    #     d /= np.linalg.norm(d) 
    #     pd = p + d 
        # ax.plot([p[0], pd[0]], [p[1], pd[1]], [p[2], pd[2]], 'r')

    # ax.plot(pnts[:, 0], pnts[:, 1], pnts[:, 2], 'c')
    
    plt.show()

