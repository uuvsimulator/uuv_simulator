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

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <limits>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>

#include <math.h>

#include "uuv_gazebo_plugins/ThrusterPlugin.hh"
#include "uuv_gazebo_plugins/Def.hh"


GZ_REGISTER_MODEL_PLUGIN(gazebo::ThrusterPlugin)

namespace gazebo {

/////////////////////////////////////////////////
ThrusterPlugin::ThrusterPlugin() : thrustForce(0),
  inputCommand(0),
  clampMin(std::numeric_limits<double>::lowest()),
  clampMax(std::numeric_limits<double>::max()),
  thrustMin(std::numeric_limits<double>::lowest()),
  thrustMax(std::numeric_limits<double>::max()),
  gain(1.0),
  isOn(true),
  thrustEfficiency(1.0),
  propellerEfficiency(1.0),
  thrusterID(-1)
{
}

/////////////////////////////////////////////////
ThrusterPlugin::~ThrusterPlugin()
{
  if (this->updateConnection)
  {
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  }
}

/////////////////////////////////////////////////
void ThrusterPlugin::Load(physics::ModelPtr _model,
                          sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Invalid model pointer");

  // Initializing the transport node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_model->GetWorld()->GetName());

  // Retrieve the link name on which the thrust will be applied
  GZ_ASSERT(_sdf->HasElement("linkName"), "Could not find linkName.");
  std::string linkName = _sdf->Get<std::string>("linkName");
  this->thrusterLink = _model->GetLink(linkName);
  GZ_ASSERT(this->thrusterLink, "thruster link is invalid");

  // Reading thruster ID
  GZ_ASSERT(_sdf->HasElement("thrusterID"), "Thruster ID was not provided");
  this->thrusterID = _sdf->Get<int>("thrusterID");

  // Thruster dynamics configuration:
  GZ_ASSERT(_sdf->HasElement("dynamics"), "Could not find dynamics.");
  this->thrusterDynamics.reset(
        DynamicsFactory::GetInstance().CreateDynamics(
          _sdf->GetElement("dynamics")));

  // Thrust conversion function
  GZ_ASSERT(_sdf->HasElement("conversion"), "Could not find dynamics");
  this->conversionFunction.reset(
        ConversionFunctionFactory::GetInstance().CreateConversionFunction(
          _sdf->GetElement("conversion")));

  // Optional paramters:
  // Rotor joint, used for visualization if available.
  if (_sdf->HasElement("jointName"))
    this->joint = _model->GetJoint(_sdf->Get<std::string>("jointName"));

  // Clamping interval
  if (_sdf->HasElement("clampMin"))
    this->clampMin = _sdf->Get<double>("clampMin");

  if (_sdf->HasElement("clampMax"))
    this->clampMax = _sdf->Get<double>("clampMax");

  if (this->clampMin >= this->clampMax)
  {
    gzmsg << "clampMax must be greater than clampMin, returning to default values..." << std::endl;
    this->clampMin = std::numeric_limits<double>::lowest();
    this->clampMax = std::numeric_limits<double>::max();
  }

  // Thrust force interval
  if (_sdf->HasElement("thrustMin"))
    this->thrustMin = _sdf->Get<double>("thrustMin");

  if (_sdf->HasElement("thrustMax"))
    this->thrustMax = _sdf->Get<double>("thrustMax");

  if (this->thrustMin >= this->thrustMax)
  {
    gzmsg << "thrustMax must be greater than thrustMin, returning to default values..." << std::endl;
    this->thrustMin = std::numeric_limits<double>::lowest();
    this->thrustMax = std::numeric_limits<double>::max();
  }

  // Gain (1.0 by default)
  if (_sdf->HasElement("gain"))
    this->gain = _sdf->Get<double>("gain");

  if (_sdf->HasElement("thrust_efficiency"))
  {
    this->thrustEfficiency = _sdf->Get<double>("thrust_efficiency");
    if (this->thrustEfficiency < 0.0 || this->thrustEfficiency > 1.0)
    {
      gzmsg << "Invalid thrust efficiency factor, setting it to 100%"
        << std::endl;
      this->thrustEfficiency = 1.0;
    }
  }

  if (_sdf->HasElement("propeller_efficiency"))
  {
    this->propellerEfficiency = _sdf->Get<double>("propeller_efficiency");
    if (this->propellerEfficiency < 0.0 || this->propellerEfficiency > 1.0)
    {
      gzmsg <<
        "Invalid propeller dynamics efficiency factor, setting it to 100%"
        << std::endl;
      this->propellerEfficiency = 1.0;
    }
  }
  // Root string for topics
  std::stringstream strs;
  strs << "/" << _model->GetName() << "/thrusters/" << this->thrusterID << "/";
  this->topicPrefix = strs.str();

  // Advertise the thrust topic
  this->thrustTopicPublisher =
      this->node->Advertise<msgs::Vector3d>(this->topicPrefix + "thrust");

  // Subscribe to the input signal topic

  this->commandSubscriber =
    this->node->Subscribe(this->topicPrefix + "input",
        &ThrusterPlugin::UpdateInput,
        this);

  // Connect the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ThrusterPlugin::Update,
                    this, _1));
}

/////////////////////////////////////////////////
void ThrusterPlugin::Init()
{
}

/////////////////////////////////////////////////
void ThrusterPlugin::Reset()
{
    this->thrusterDynamics->Reset();
}

/////////////////////////////////////////////////
void ThrusterPlugin::Update(const common::UpdateInfo &_info)
{
  GZ_ASSERT(!std::isnan(this->inputCommand),
            "nan in this->inputCommand");

  double dynamicsInput;
  double dynamicState;
  // Test if the thruster has been turned off
  if (this->isOn)
  {
    double clamped = this->inputCommand;
    clamped = std::min(clamped, this->clampMax);
    clamped = std::max(clamped, this->clampMin);

    dynamicsInput = this->gain*clamped;
  }
  else
  {
    // In case the thruster is turned off in runtime, the dynamic state
    // will converge to zero
    dynamicsInput = 0.0;
  }
  dynamicState = this->propellerEfficiency *
    this->thrusterDynamics->update(dynamicsInput, _info.simTime.Double());

  GZ_ASSERT(!std::isnan(dynamicState), "Invalid dynamic state");
  // Multiply the output force magnitude with the efficiency
  this->thrustForce = this->thrustEfficiency *
    this->conversionFunction->convert(dynamicState);
  GZ_ASSERT(!std::isnan(this->thrustForce), "Invalid thrust force");

  // Use the thrust force limits
  this->thrustForce = std::max(this->thrustForce, this->thrustMin);
  this->thrustForce = std::min(this->thrustForce, this->thrustMax);

  this->thrustForceStamp = _info.simTime;
  math::Vector3 force(this->thrustForce, 0, 0);

  this->thrusterLink->AddRelativeForce(force);

  if (this->joint)
  {
    // Let joint rotate with correct angular velocity.
    this->joint->SetVelocity(0, dynamicState);
  }

  // Publish thrust:
  msgs::Vector3d thrustMsg;
  msgs::Set(&thrustMsg, ignition::math::Vector3d(this->thrustForce, 0., 0.));
  this->thrustTopicPublisher->Publish(thrustMsg);
}

/////////////////////////////////////////////////
void ThrusterPlugin::UpdateInput(ConstDoublePtr &_msg)
{
  this->inputCommand = _msg->value();
}
}
