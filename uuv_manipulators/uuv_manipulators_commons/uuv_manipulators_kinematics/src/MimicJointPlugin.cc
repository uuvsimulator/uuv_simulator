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

#include <uuv_manipulators_kinematics/MimicJointPlugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

namespace gazebo
{
MimicJointPlugin::MimicJointPlugin()
{
  this->killSimulation = false;
  this->joint.reset();
  this->mimicJoint.reset();
}

MimicJointPlugin::~MimicJointPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

  this->killSimulation = true;
}

void MimicJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Error message if the model couldn't be found
  if (!this->model)
  {
    ROS_ERROR("Parent model is NULL! Plugin could not be loaded.");
    return;
  }

  // Check that ROS has been initialized
  if (!ros::isInitialized())
  {
    ROS_ERROR("A ROS node for Gazebo has not been initialized,"
        " unable to load plugin.");
    return;
  }

  // Check for joint element
  if (!_sdf->HasElement("joint"))
  {
    ROS_ERROR("No joint element present. MimicJointPlugin could"
      " not be loaded.");
    return;
  }

  jointName = _sdf->GetElement("joint")->Get<std::string>();

  // Check for mimicJoint element
  if (!_sdf->HasElement("mimicJoint"))
  {
    ROS_ERROR("No mimicJoint element present. MimicJointPlugin "
      "could not be loaded.");
    return;
  }

  this->mimicJointName = _sdf->GetElement("mimicJoint")->Get<std::string>();

  double p, i, d;

  if (_sdf->HasElement("kp"))
    p = _sdf->Get<double>("kp");
  else
    p = 0.0;

  if (_sdf->HasElement("kd"))
    d = _sdf->Get<double>("kd");
  else
    d = 0.0;

  if (_sdf->HasElement("ki"))
    i = _sdf->Get<double>("ki");
  else
    i = 0.0;

  this->pid = control_toolbox::Pid(p, i, d);

  // Check for this->multiplier element
  this->multiplier = 1.0;
  if (_sdf->HasElement("multiplier"))
    this->multiplier = _sdf->GetElement("multiplier")->Get<double>();

  // Check for offset element
  this->offset = 0.0;
  if (_sdf->HasElement("offset"))
    this->offset = _sdf->GetElement("offset")->Get<double>();

  // Check for sensitiveness element
  this->sensitiveness = 0.0;
  if (_sdf->HasElement("sensitiveness"))
    this->sensitiveness = _sdf->GetElement("sensitiveness")->Get<double>();

  // Get pointers to joints
  this->joint = this->model->GetJoint(jointName);
  if (!this->joint)
  {
    ROS_ERROR("No joint named %s. MimicJointPlugin could "
      "not be loaded.", jointName.c_str());
    return;
  }
  this->mimicJoint = this->model->GetJoint(mimicJointName);
  if (!this->mimicJoint)
  {
    ROS_ERROR("No (mimic) joint named %s. MimicJointPlugin could "
      "not be loaded.", mimicJointName.c_str());
    return;
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MimicJointPlugin::UpdateChild, this));
}

void MimicJointPlugin::UpdateChild()
{
  static ros::Duration period(
    this->world->GetPhysicsEngine()->GetMaxStepSize());

  // Set mimic joint's angle based on joint's angle
  double angle = this->joint->GetAngle(0).Radian() *
    this->multiplier + this->offset;

  if (abs(angle - this->mimicJoint->GetAngle(0).Radian()) >=
    this->sensitiveness)
  {
    double a = this->mimicJoint->GetAngle(0).Radian();
    double error = angle - a;
    double effort = gazebo::math::clamp(this->pid.computeCommand(error, period),
      -this->mimicJoint->GetEffortLimit(0),
      this->mimicJoint->GetEffortLimit(0));

    this->mimicJoint->SetForce(0,
      this->mimicJoint->CheckAndTruncateForce(0, effort));
  }
}

GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)
}
