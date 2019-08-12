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
from __future__ import print_function
import os
import time
import sys, select, termios, tty
import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Accel, Vector3

class KeyBoardVehicleTeleop:
    def __init__(self):
        # Class Variables
        self.settings = termios.tcgetattr(sys.stdin)

        # Speed setting
        self.speed = 1 # 1 = Slow, 2 = Fast
        self.l = Vector3(0, 0, 0) # Linear Velocity for Publish
        self.a = Vector3(0, 0, 0) # Angular Velocity for publishing
        self.linear_increment = 0.05 # How much to increment linear velocities by, to avoid jerkyness
        self.linear_limit = 1 # Linear velocity limit = self.linear_limit * self.speed
        self.angular_increment = 0.05
        self.angular_limit = 0.5
        # User Interface
        self.msg = """
    Control Your Vehicle!
    ---------------------------
    Moving around:
        W/S: X-Axis
        A/D: Y-Axis
        X/Z: Z-Axis

        Q/E: Yaw
        I/K: Pitch
        J/L: Roll

    Slow / Fast: 1 / 2

    CTRL-C to quit
            """

        # Default message remains as twist
        self._msg_type = 'twist'
        if rospy.has_param('~type'):
            self._msg_type = rospy.get_param('~type')
            if self._msg_type not in ['twist', 'accel']:
                raise rospy.ROSException('Teleoperation output must be either '
                                         'twist or accel')
        # Name Publisher topics accordingly
        if self._msg_type == 'twist':
            self._output_pub = rospy.Publisher('output', Twist, queue_size=1)
        else:
            self._output_pub = rospy.Publisher('output', Accel, queue_size=1)

        print(self.msg)

        # Ros Spin
        rate = rospy.Rate(50)  # 50hz
        while not rospy.is_shutdown():
            rate.sleep()
            self._parse_keyboard()

    # Every spin this function will return the key being pressed
    # Only works for one key per spin currently, thus limited control exploring alternative methods
    def _get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        return key


    # Function to gradually build up the speed and avoid jerkyness #
    def _speed_windup(self, speed, increment, limit, reverse):
        if reverse == True:
            speed -= increment * self.speed
            if speed < -limit * self.speed:
                speed = -limit * self.speed
        else:
            speed += increment * self.speed
            if speed > limit * self.speed:
                speed = limit * self.speed

        return speed

    def _parse_keyboard(self):
        # Save key peing pressed
        key_press = self._get_key()

        # Set Vehicle Speed #
        if key_press == "1":
            self.speed = 1
        if key_press == "2":
            self.speed = 2

        # Choose ros message accordingly
        if self._msg_type == 'twist':
            cmd = Twist()
        else:
            cmd = Accel()

        # If a key is pressed assign relevent linear / angular vel
        if key_press!='':
            # Linear velocities:
            # Forward
            if key_press == "w":
                self.l.x = self._speed_windup(self.l.x, self.linear_increment, self.linear_limit, False)
            # Backwards
            if key_press == "s":
                self.l.x = self._speed_windup(self.l.x, self.linear_increment, self.linear_limit, True)
            # Left
            if key_press == "a":
                self.l.y = self._speed_windup(self.l.y, self.linear_increment, self.linear_limit, False)
            # Right
            if key_press == "d":
                self.l.y = self._speed_windup(self.l.y, self.linear_increment, self.linear_limit, True)
            # Up
            if key_press == "x":
                self.l.z = self._speed_windup(self.l.z, self.linear_increment, self.linear_limit*0.5, False)
            # Down
            if key_press == "z":
                self.l.z = self._speed_windup(self.l.z, self.linear_increment, self.linear_limit*0.5, True)

            # Angular Velocities
            # Roll Left
            if key_press == "j":
                self.a.x = self._speed_windup(self.a.x, self.linear_increment, self.linear_limit, True)
            # Roll Right
            if key_press == "l":
                self.a.x = self._speed_windup(self.a.x, self.linear_increment, self.linear_limit, False)
            # Pitch Down
            if key_press == "i":
                self.a.y = self._speed_windup(self.a.y, self.linear_increment, self.linear_limit, False)
            # Pitch Up
            if key_press == "k":
                self.a.y = self._speed_windup(self.a.y, self.linear_increment, self.linear_limit, True)
            # Yaw Left
            if key_press == "q":
                self.a.z = self._speed_windup(self.a.z, self.linear_increment, self.linear_limit, False)
            # Yaw Right
            if key_press == "e":
                self.a.z = self._speed_windup(self.a.z, self.linear_increment, self.linear_limit, True)

        else:
            # If no button is pressed reset velocities to 0
            self.l = Vector3(0, 0, 0)
            self.a = Vector3(0, 0, 0)

        # Store velocity message into Twist format
        cmd.angular = self.a
        cmd.linear = self.l

        # If ctrl+c kill node
        if (key_press == '\x03'):
            rospy.loginfo('Keyboard Interrupt Pressed')
            rospy.loginfo('Shutting down [%s] node' % node_name)

            # Set twists to 0
            cmd.angular = Vector3(0, 0, 0)
            cmd.linear = Vector3(0, 0, 0)
            self._output_pub.publish(cmd)

            exit(-1)

        # Publish message
        self._output_pub.publish(cmd)

if __name__ == '__main__':

    # Wait for 5 seconds, so the instructions are the last thing to print in terminal
    time.sleep(5)
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    teleop = KeyBoardVehicleTeleop()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    rospy.loginfo('Shutting down [%s] node' % node_name)
