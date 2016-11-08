// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This source code is derived from roboticsgroup_gazebo_plugins
//   (https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins)
// Copyright (c) 2014, Konstantinos Chatzilygeroudis, licensed under the
// BSD 3-Clause license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.

#ifndef __MIMIC_JOINT_PLUGIN_HH__
#define __MIMIC_JOINT_PLUGIN_HH__

#include <string>

// PID controller
#include <control_toolbox/pid.h>

// Boost includes
#include <boost/bind.hpp>

// Gazebo includes
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class MimicJointPlugin : public ModelPlugin
  {
    public: MimicJointPlugin();

    public: ~MimicJointPlugin();

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    public: void UpdateChild();

    private: std::string jointName;

    private: std::string mimicJointName;

    private: double multiplier;

    private: double offset;

    private: double sensitiveness;

    private: double maxEffort;

    private: bool killSimulation;
    // PID controller if needed
    private: control_toolbox::Pid pid;
    // Pointers to the joints
    private: physics::JointPtr joint;

    private: physics::JointPtr mimicJoint;
    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the world
    private: physics::WorldPtr world;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };
}
#endif  // __MIMIC_JOINT_PLUGIN_HH__
