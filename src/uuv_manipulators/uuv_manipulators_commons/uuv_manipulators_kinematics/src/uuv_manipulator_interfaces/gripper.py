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

import rospy

import numpy as np
import PyKDL
from uuv_manipulator_interfaces.kin_chain import KinChainInterface
from uuv_manipulators_msgs.msg import EndeffectorState
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time


class GripperInterface(object):
    TYPE = ['no_gripper', 'parallel', 'jaw']
    STATE = ['ready', 'disabled', 'moving']

    def __init__(self, namespace=None, arm_name=None):
        if None in [namespace, arm_name]:
            ns = [item for item in rospy.get_namespace().split('/') if len(item) > 0]
            if len(ns) != 2:
                rospy.ROSException('The controller must be called inside the manipulator namespace')

            self._namespace = ns[0]
            self._arm_name = ns[1]
        else:
            self._namespace = namespace
            self._arm_name = arm_name

        if self._namespace[0] != '/':
            self._namespace = '/' + self._namespace

        if self._namespace[-1] != '/':
            self._namespace += '/'

        if not rospy.has_param('~gripper'):
            raise rospy.ROSException('Gripper configuration not given')

        # Retrieve gripper configuration information
        self._gripper_config = rospy.get_param('~gripper')

        # Retrive gripper type
        if 'type' not in self._gripper_config:
            raise ROSException('Gripper type not defined')

        if self._gripper_config['type'] not in self.TYPE:
            raise ROSException('Gripper type not defined')

        self._gripper_type = self._gripper_config['type']

        self._closed_limit = 'upper'
        if 'closed_limit' in self._gripper_config:
            self._closed_limit = self._gripper_config['closed_limit']

        self._full_open_limit = 'upper'
        if 'full_open_limit' in self._gripper_config:
            self._full_open_limit = self._gripper_config['full_open_limit']

        rospy.loginfo('Gripper type=' + self._gripper_type)

        # Chain groups
        self._groups = dict()

        # Loading the kinematic chain for each finger
        for group in self._gripper_config['groups']:
            chain = self._gripper_config['groups'][group]
            self._groups[group] = KinChainInterface(name=group,
                                                    base=self._gripper_config['base'],
                                                    ee_link=chain['ee'],
                                                    namespace=self._namespace,
                                                    arm_name=self._arm_name,
                                                    compute_fk_for_all=False)

        # Published topics
        self._pubTopics = dict()
        # Controller input topics
        self._pubControllers = dict()
        # Subscribed topics
        self._subTopics = dict()

        # Subscribe to the joint states topic
        self._subTopics['joint_states'] = rospy.Subscriber(
            self._namespace + 'joint_states',
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

        # Publish end-effector state
        self._pubTopics['state'] = rospy.Publisher(
            self._namespace + self._arm_name + '/ee_state',
            EndeffectorState,
            queue_size=1)

        # Set gripper state
        self._gripper_state = 'ready'
        # Retrieving the name of the control joint
        self._control_joint = None
        self._control_joint_group = None

        self._mimic_joint = None
        self._mimic_joint_group = None

        if 'control_joint' in self._gripper_config:
            # Correct the name including the namespace in TF
            for name in self._groups:
                for joint in self._groups[name].joint_names:
                    if self._gripper_config['control_joint'] in joint:
                        self._control_joint = joint
                        self._control_joint_group = name
                    if self._gripper_config['mimic_joint'] in joint:
                        self._mimic_joint = joint
                        self._mimic_joint_group = name

            rospy.loginfo(self._control_joint_group + ' - ' + self._control_joint)

        for joint in self.get_joint_names():
            self._pubControllers[joint] = rospy.Publisher(
                self._namespace + joint + '/controller/command',
                Float64,
                queue_size=1)

        # Publishing rate for state topics
        self._publish_rate = 50
        # Set timer to regularly publish the state
        self._publish_state_timer = rospy.Timer(
            rospy.Duration(1.0 / self._publish_rate),
            self._publish_endeffector_state)

        rospy.on_shutdown(self._on_shutdown)

    def __del__(self):
        """Class destructor."""
        for topic in self._subTopics:
            self._subTopics[topic].unregister()
        for topic in self._pubTopics:
            self._pubTopics[topic].unregister()

    @property
    def full_namespace(self):
        return self._namespace + self._arm_name

    @property
    def is_fully_open(self):
        if self._control_joint is None:
            return None
        else:
            return self.control_joint_position == self.control_joint_limits[self._full_open_limit]

    @property
    def is_closed(self):
        if self._control_joint is None:
            return None
        else:
            return self.control_joint_position == self.control_joint_limits[self._closed_limit]

    @property
    def closed_pos(self):
        if self._control_joint is None:
            return None
        else:
            return self.control_joint_limits[self._closed_limit]

    @property
    def fully_open_pos(self):
        if self._control_joint is None:
            return None
        else:
            return self.control_joint_limits[self._full_open_limit]

    @property
    def is_moving(self):
        if self.control_joint is None:
            return None
        else:
            return self.control_joint_velocity > 0.001

    @property
    def is_ready(self):
        return self._gripper_state == 'ready'

    @property
    def is_disabled(self):
        return self._gripper_state == 'disabled'

    @property
    def is_parallel(self):
        return self._gripper_type == 'parallel'

    @property
    def is_jaw(self):
        return self._gripper_type == 'jaw'

    @property
    def control_joint(self):
        return self._control_joint

    @property
    def control_joint_group(self):
        return self._control_joint_group

    @property
    def control_joint_position(self):
        if None in [self._control_joint, self._control_joint_group]:
            return None
        return self._groups[self._control_joint_group].joint_angles[self._control_joint]

    @property
    def control_joint_velocity(self):
        if None in [self._control_joint, self._control_joint_group]:
            return None
        return self._groups[self._control_joint_group].joint_velocities[self._control_joint]

    @property
    def control_joint_effort(self):
        if None in [self._control_joint, self._control_joint_group]:
            return None
        return self._groups[self._control_joint_group].joint_efforts[self._control_joint]

    @property
    def control_joint_limits(self):
        if None in [self._control_joint, self._control_joint_group]:
            return None
        return self._groups[self._control_joint_group].joint_limits[self._control_joint]

    @property
    def control_joint_pos_ratio(self):
        limit = self.control_joint_limits
        cur_ratio = self.get_position_ratio(self.control_joint_position)
        return cur_ratio

    @property
    def mimic_joint(self):
        return self._mimic_joint

    @property
    def mimic_joint_group(self):
        return self._mimic_joint_group

    @property
    def mimic_joint_position(self):
        if None in [self._mimic_joint, self._mimic_joint_group]:
            return None
        return self._groups[self._mimic_joint_group].joint_angles[self._mimic_joint]

    @property
    def mimic_joint_velocity(self):
        if None in [self._mimic_joint, self._mimic_joint_group]:
            return None
        return self._groups[self._mimic_joint_group].joint_velocities[self._mimic_joint]

    @property
    def mimic_joint_effort(self):
        if None in [self._mimic_joint, self._mimic_joint_group]:
            return None
        return self._groups[self._mimic_joint_group].joint_efforts[self._mimic_joint]

    @property
    def mimic_joint_limits(self):
        if None in [self._mimic_joint, self._mimic_joint_group]:
            return None
        return self._groups[self._mimic_joint_group].joint_limits[self._mimic_joint]

    @property
    def mimic_joint_pos_ratio(self):
        limit = self.mimic_joint_limits
        cur_ratio = self.get_position_ratio(self.mimic_joint_position)
        return cur_ratio

    def _on_shutdown(self):
        self._publish_state_timer.shutdown()

    def _on_joint_states(self, msg):
        for name in self._groups:
            # Store the joint states
            self._groups[name].update_joint_states(msg)

    def _publish_endeffector_state(self, event):
        msg = EndeffectorState()
        msg.stamp = rospy.get_rostime()
        msg.state = self._gripper_state.lower()
        self._pubTopics['state'].publish(msg)

    def get_position_ratio(self, pos):
        return (pos - self.closed_pos) / (self.fully_open_pos - self.closed_pos)

    def get_type(self):
        return self._gripper_type.name

    def get_joint_names(self, group=None):
        if group is None:
            joint_names = list()
            for name in self._groups:
                joints = self._groups[name].joint_names
                for joint in joints:
                    if joint not in joint_names:
                        joint_names.append(joint)
            return joint_names
        else:
            if group not in self._groups:
                return list()
            else:
                return self._groups[group].joint_names

    def get_group_names(self):
        return self._groups.keys()

    def set_state(self, state):
        for gripper_state in GripperState:
            if state == gripper_state.name:
                self._gripper_state = gripper_state
                return True
        return False

    def get_group(self, name):
        if name not in self._groups:
            return None
        return self._groups[name]

    def set_command(self, effort):
        if self._control_joint is None:
            return
        set_point = effort
        if set_point < -self.control_joint_limits['effort']:
            set_point = -self.control_joint_limits['effort']
        elif set_point > self.control_joint_limits['effort']:
            set_point = self.control_joint_limits['effort']

        self._pubControllers[self._control_joint].publish(set_point)
        self._pubControllers[self._mimic_joint].publish(float(self._gripper_config['mimic_joint_gain']) * set_point)

    def set_controller_command(self, joint, effort):
        if joint not in self._pubControllers:
            raise rospy.ROSException('Invalid joint tag')
        self._pubControllers[joint].publish(effort)
