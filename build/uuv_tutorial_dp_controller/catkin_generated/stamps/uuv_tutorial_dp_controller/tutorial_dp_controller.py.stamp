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

# Import the ROS Python library
import rospy
# Import the NumPy package for numerical computations
import numpy as np
# Import the dynamic positioning controller base class to inherit methods such as error computation update,
# publishing from some important ROS topics (e.g. trajectory, pose and velocity reference) and access to
# the vehicle model class, that is necessary to explicitly use the vehicle model
from uuv_control_interfaces import DPControllerBase


class TutorialDPController(DPControllerBase):
    # A new controller that is based on the DPControllerBase must at least provide the implementation of
    # the method update_controller.
    # The _reset_controller method can also be overridden and it will be called every time there is a service call
    # <vehicle namespace>/reset_controller. The default implementation sets the reference and error vectors to
    # zero.
    # The update_controller method must contain the implementation of the control algorithm and will be called
    # by every update of the vehicle's odometry message. It is therefore not necessary to explicitly call this update
    # function in this controller implementation.
    # For the controller to send the control torques to the vehicle's thruster manager, at the end of the
    # update_controller function the 6 x 1 control vector (type numpy.ndarray) sent using the function
    # publish_control_wrench from the super class, which will generate a Wrench ROS message and publish it to the
    # correspondent thruster manager node.
    # For this tutorial, a simple PID controller will be implemented. The controller's control torque output is
    # "tau" therefore computed as:
    #
    #   tau = Kp * e + Kd * de/dt + Ki int_e
    #
    # where e is the pose error vector, in this case defined as e = (x, y, z, roll, pitch, yaw)^T

    def __init__(self):
        # Calling the constructor of the super-class DPControllerBase, which has the implementation of the error
        # computation update and to publish the resulting torque control vector.
        super(TutorialDPController, self).__init__(self)

        # The controller should read its parameters from the ROS parameter server for initial setup
        # One way to do this is to read the parameters from the node's private parameter namespace, which is done by
        # reading the parameter tag with an "~" at the beginning. If this method is used, the parameters should be
        # initialized accordingly in the controller startup launch file, such as
        #
        # <launch>
        #   <node pkg="example_package" type="example_node.py" name="example_node" output="screen">
        #       <rosparam>
        #           param_1: 0.0
        #           param_2: 0.0
        #       </rosparam>
        #   </node>
        # </launch>
        #
        # For more information, see http://wiki.ros.org/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle

        # Let's initialize the controller gain matrices Kp, Kd and Ki
        self._Kp = np.zeros(shape=(6, 6))
        self._Kd = np.zeros(shape=(6, 6))
        self._Ki = np.zeros(shape=(6, 6))
        # Initialize the integrator component
        self._int = np.zeros(shape=(6,))
        # Initialize variable that will store the vehicle pose error
        self._error_pose = np.zeros(shape=(6,))

        # Now the gain matrices need to be set according to the variables stored in the parameter server
        # For simplicity, the gain matrices are defined as diagonal matrices, so only 6 coefficients are
        # needed
        if rospy.get_param('~Kp'):
            Kp_diag = rospy.get_param('~Kp')
            if len(Kp_diag) == 6:
                self._Kp = np.diag(Kp_diag)
            else:
                # If the vector provided has the wrong dimension, raise an exception
                raise rospy.ROSException('For the Kp diagonal matrix, 6 coefficients are needed')

        # Do the same for the other two matrices
        if rospy.get_param('~Kd'):
            diag = rospy.get_param('~Kd')
            if len(diag) == 6:
                self._Kd = np.diag(diag)
                print 'Kd=\n', self._Kd
            else:
                # If the vector provided has the wrong dimension, raise an exception
                raise rospy.ROSException('For the Kd diagonal matrix, 6 coefficients are needed')

        if rospy.get_param('~Ki'):
            diag = rospy.get_param('~Ki')
            if len(diag) == 6:
                self._Ki = np.diag(diag)
                print 'Ki=\n', self._Ki
            else:
                # If the vector provided has the wrong dimension, raise an exception
                raise rospy.ROSException('For the Ki diagonal matrix, 6 coefficients are needed')
            self._is_init = True

    def _reset_controller(self):
        # The _reset_controller method from the super class DPControllerBase already sets the error
        # and reference vectors to zero, but this class has additional attributes that should also
        # be taken care of.
        # This implementation will, therefore, first call the super class reset method
        super(TutorialDPController, self)._reset_controller()
        # And then proceed to set the internal variables back to zero
        self._error_pose = np.zeros(shape=(6,))
        self._int = np.zeros(shape=(6,))

    def update_controller(self):
        if not self._is_init:
            return False
        # The controller algorithm must be implemented here, the super class will connect this method
        # to the odometry update as a callback function

        # First test whether or not the odometry topic subscriber has already been initialized
        if not self.odom_is_init:
            return

        # Update the integrator, read the super class vector for the pose error (orientation is represented
        # with Euler angles in RPY convention) and integrate to the stored pose error from the last
        # iteration
        self._int = self._int + 0.5 * (self.error_pose_euler + self._error_pose) * self._dt

        # Store the current pose error for the next iteration
        self._error_pose = self.error_pose_euler

        # Compute the control forces and torques using the current error vectors available
        tau = np.dot(self._Kp, self.error_pose_euler) + np.dot(self._Kd, self._errors['vel']) + \
            np.dot(self._Ki, self._int)

        # Use the super class method to convert the control force vector into a ROS message
        # and publish it as an input to the vehicle's thruster manager. The thruster manager module
        # will then distribute the efforts amongst the thrusters using the thruster allocation matrix
        self.publish_control_wrench(tau)

if __name__ == '__main__':
    # Since this is an ROS node, this Python script has to be treated as an executable
    # Remember to convert this Python file into an executable. This can be done with
    #
    # cd <path_to_ros_package>/scripts
    # chmod 777 tutorial_dp_controller.py
    #
    # This file has also to be included in this package's CMakeLists.txt
    # After the line catkin_package() in the CMakeLists.txt, include the following
    #
    # catkin_install_python(PROGRAMS scripts/tutorial_dp_controller.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

    print('Tutorial - DP Controller')
    rospy.init_node('tutorial_dp_controller')

    try:
        node = TutorialDPController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
