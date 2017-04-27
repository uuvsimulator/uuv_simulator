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

import roslib
import numpy as np
import matplotlib.pyplot as plt
import uuv_trajectory_generator
import time
from geometry_msgs.msg import Point
from mpl_toolkits.mplot3d import Axes3D

roslib.load_manifest('uuv_trajectory_control')

"""
Demo file to demonstrate the waypoint interpolation method with generation of
velocity and acceleration profile using a constant rate.
"""

def run_generator(waypoint_set):
    # Initialize the trajectory generator
    gen = uuv_trajectory_generator.WPTrajectoryGenerator(full_dof=False)
    gen.init_waypoints(wp_set)

    dt = 0.05
    idx = 0
    pnts = list()
    avg_time = 0.0

    gen.set_start_time(0)
    for ti in np.arange(-2, gen.get_max_time(), dt):
        tic = time.clock()
        pnts.append(gen.interpolate(ti))
        toc = time.clock()
        avg_time += toc - tic
        idx += 1
    avg_time /= idx
    print 'Average processing time [s] =', avg_time
    fig = plt.figure()
    # Trajectory and heading 3D plot
    ax = fig.add_subplot(111, projection='3d')

    # Plot the generated path
    ax.plot([p.x for p in pnts], [p.y for p in pnts], [p.z for p in pnts], 'b')

    # Plot original waypoints
    ax.plot(wp_set.x, wp_set.y, wp_set.z, 'r.')

    # Plot the raw path along the waypoints
    ax.plot(wp_set.x, wp_set.y, wp_set.z, 'g--')

    for i in range(1, len(pnts), 100):
        p0 = pnts[i - 1]
        p1 = p0.pos + np.dot(p0.rot_matrix, [2, 0, 0])
        ax.plot([p0.pos[0], p1[0]], [p0.pos[1], p1[1]], [p0.pos[2], p1[2]], 'c', linewidth=2)
    ax.grid(True)

    # Position and orientation plot
    fig = plt.figure()
    ax = fig.add_subplot(211)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.pos[i] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Position')
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    ax = fig.add_subplot(212)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.rot[i] * 180 / np.pi for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Rotation')
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    fig = plt.figure()
    ax = fig.add_subplot(211)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.vel[i] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Linear velocity')
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    ax = fig.add_subplot(212)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.vel[i+3] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Angular velocity')
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    fig = plt.figure()
    ax = fig.add_subplot(211)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.acc[i] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Linear accelerations')
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    ax = fig.add_subplot(212)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.acc[i+3] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Angular accelerations')
    ax.set_xlim(pnts[0].t, pnts[-1].t)

if __name__ == '__main__':
    # For a helical trajectory
    wp_set = uuv_trajectory_generator.WaypointSet()
    wp_set.generate_helix(radius=8,
                          center=Point(2, 2, -30),
                          num_points=50,
                          max_forward_speed=0.5,
                          delta_z=10,
                          num_turns=1.2,
                          theta_offset=0.0,
                          heading_offset=0.0)
    # Add some waypoints at the beginning
    wp_set.add_waypoint(uuv_trajectory_generator.Waypoint(-10, -12, -36, 0.5),
                        add_to_beginning=True)
    wp_set.add_waypoint(uuv_trajectory_generator.Waypoint(-13, -15, -44, 0.5),
                        add_to_beginning=True)
    wp_set.add_waypoint(uuv_trajectory_generator.Waypoint(-20, -24, -48, 0.5),
                        add_to_beginning=True)
    wp_set.add_waypoint(uuv_trajectory_generator.Waypoint(-10, 10, -5, 0.5))
    wp_set.add_waypoint(uuv_trajectory_generator.Waypoint(-20, 20, -5, 0.5))
    run_generator(wp_set)

    plt.show()
