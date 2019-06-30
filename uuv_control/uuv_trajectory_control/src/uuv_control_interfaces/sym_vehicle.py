# Copyright (c) 2016-2019 The UUV Simulator Authors.
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
from .vehicle import Vehicle, cross_product_operator
from uuv_thrusters import ThrusterManager
from uuv_auv_actuator_interface import ActuatorManager
try: 
    import casadi
    CASADI_IMPORTED = True
except ImportError:
    CASADI_IMPORTED = False


class SymVehicle(Vehicle):
    def __init__(self, inertial_frame_id='world'):
        Vehicle.__init__(self, inertial_frame_id)
    
        if CASADI_IMPORTED:
            # Declaring state variables
            ## Generalized position vector
            self.eta = casadi.SX.sym('eta', 6)
            ## Generalized velocity vector
            self.nu = casadi.SX.sym('nu', 6)

            # Build the Coriolis matrix
            self.CMatrix = casadi.SX.zeros(6, 6)

            S_12 = - cross_product_operator(
                casadi.mtimes(self._Mtotal[0:3, 0:3], self.nu[0:3]) +
                casadi.mtimes(self._Mtotal[0:3, 3:6], self.nu[3:6]))
            S_22 = - cross_product_operator(
                casadi.mtimes(self._Mtotal[3:6, 0:3], self.nu[0:3]) +
                casadi.mtimes(self._Mtotal[3:6, 3:6], self.nu[3:6]))

            self.CMatrix[0:3, 3:6] = S_12
            self.CMatrix[3:6, 0:3] = S_12
            self.CMatrix[3:6, 3:6] = S_22

            # Build the damping matrix (linear and nonlinear elements)
            self.DMatrix = - casadi.diag(self._linear_damping)        
            self.DMatrix -= casadi.diag(self._linear_damping_forward_speed)
            self.DMatrix -= casadi.diag(self._quad_damping * self.nu)      

            # Build the restoring forces vectors wrt the BODY frame
            Rx = np.array([[1, 0, 0],
                        [0, casadi.cos(self.eta[3]), -1 * casadi.sin(self.eta[3])],
                        [0, casadi.sin(self.eta[3]), casadi.cos(self.eta[3])]])
            Ry = np.array([[casadi.cos(self.eta[4]), 0, casadi.sin(self.eta[4])],
                        [0, 1, 0],
                        [-1 * casadi.sin(self.eta[4]), 0, casadi.cos(self.eta[4])]])
            Rz = np.array([[casadi.cos(self.eta[5]), -1 * casadi.sin(self.eta[5]), 0],
                        [casadi.sin(self.eta[5]), casadi.cos(self.eta[5]), 0],
                        [0, 0, 1]])

            R_n_to_b = casadi.transpose(casadi.mtimes(Rz, casadi.mtimes(Ry, Rx)))

            if inertial_frame_id == 'world_ned':
                Fg = casadi.SX([0, 0, -self.mass * self.gravity])
                Fb = casadi.SX([0, 0, self.volume * self.gravity * self.density])
            else:
                Fg = casadi.SX([0, 0, self.mass * self.gravity])
                Fb = casadi.SX([0, 0, -self.volume * self.gravity * self.density])

            self.gVec = casadi.SX.zeros(6)

            self.gVec[0:3] = -1 * casadi.mtimes(R_n_to_b, Fg + Fb)  
            self.gVec[3:6] = -1 * casadi.mtimes(
                R_n_to_b, casadi.cross(self._cog, Fg) + casadi.cross(self._cob, Fb))
            
            # Build Jacobian
            T = 1 / casadi.cos(self.eta[4]) * np.array(
                [[0, casadi.sin(self.eta[3]) * casadi.sin(self.eta[4]), casadi.cos(self.eta[3]) * casadi.sin(self.eta[4])],
                [0, casadi.cos(self.eta[3]) * casadi.cos(self.eta[4]), -casadi.cos(self.eta[4]) * casadi.sin(self.eta[3])],
                [0, casadi.sin(self.eta[3]), casadi.cos(self.eta[3])]])

            self.eta_dot = casadi.vertcat(
                casadi.mtimes(casadi.transpose(R_n_to_b), self.nu[0:3]),
                casadi.mtimes(T, self.nu[3::]))

            self.u = casadi.SX.sym('u', 6)
            
            self.nu_dot = casadi.solve(
                self._Mtotal, 
                self.u - casadi.mtimes(self.CMatrix, self.nu) - casadi.mtimes(self.DMatrix, self.nu) - self.gVec)

        