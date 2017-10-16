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

/// \file UnderwaterCurrentPlugin.cc

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

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

#include <uuv_world_plugins/UnderwaterCurrentPlugin.hh>

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(UnderwaterCurrentPlugin)

/////////////////////////////////////////////////
UnderwaterCurrentPlugin::UnderwaterCurrentPlugin()
{
  // Doing nothing for now
}

/////////////////////////////////////////////////
UnderwaterCurrentPlugin::~UnderwaterCurrentPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world != NULL, "World pointer is invalid");
  GZ_ASSERT(_sdf != NULL, "SDF pointer is invalid");

  this->world = _world;
  this->sdf = _sdf;

  // Read the namespace for topics and services
  this->ns = _sdf->Get<std::string>("namespace");

  gzmsg << "Loading underwater world..." << std::endl;
  // Initializing the transport node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  // Retrieve the current velocity configuration, if existent
  GZ_ASSERT(this->sdf->HasElement("constant_current"),
    "Constant current configuration not available");
  sdf::ElementPtr currentVelocityParams = this->sdf->GetElement(
    "constant_current");

  // Read the topic names from the SDF file
  if (currentVelocityParams->HasElement("topic"))
    this->currentVelocityTopic =
      currentVelocityParams->Get<std::string>("topic");
  else
    this->currentVelocityTopic = "current_velocity";
    
  GZ_ASSERT(!this->currentVelocityTopic.empty(),
    "Empty current velocity topic");

  if (currentVelocityParams->HasElement("velocity"))
  {
    sdf::ElementPtr elem = currentVelocityParams->GetElement("velocity");
    if (elem->HasElement("mean"))
        this->currentVelModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
        this->currentVelModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
        this->currentVelModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
        this->currentVelModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
        this->currentVelModel.noiseAmp = elem->Get<double>("noiseAmp");

    GZ_ASSERT(this->currentVelModel.min < this->currentVelModel.max,
      "Invalid current velocity limits");
    GZ_ASSERT(this->currentVelModel.mean >= this->currentVelModel.min,
      "Mean velocity must be greater than minimum");
    GZ_ASSERT(this->currentVelModel.mean <= this->currentVelModel.max,
      "Mean velocity must be smaller than maximum");
    GZ_ASSERT(this->currentVelModel.mu >= 0 && this->currentVelModel.mu < 1,
      "Invalid process constant");
    GZ_ASSERT(this->currentVelModel.noiseAmp < 1 &&
      this->currentVelModel.noiseAmp >= 0,
      "Noise amplitude has to be smaller than 1");
  }

  this->currentVelModel.var = this->currentVelModel.mean;
  gzmsg << "Current velocity [m/s] Gauss-Markov process model:" << std::endl;
  this->currentVelModel.Print();

  if (currentVelocityParams->HasElement("horizontal_angle"))
  {
    sdf::ElementPtr elem =
      currentVelocityParams->GetElement("horizontal_angle");

    if (elem->HasElement("mean"))
      this->currentHorzAngleModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
      this->currentHorzAngleModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->currentHorzAngleModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
      this->currentHorzAngleModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
      this->currentHorzAngleModel.noiseAmp = elem->Get<double>("noiseAmp");

    GZ_ASSERT(this->currentHorzAngleModel.min <
      this->currentHorzAngleModel.max,
      "Invalid current horizontal angle limits");
    GZ_ASSERT(this->currentHorzAngleModel.mean >=
      this->currentHorzAngleModel.min,
      "Mean horizontal angle must be greater than minimum");
    GZ_ASSERT(this->currentHorzAngleModel.mean <=
      this->currentHorzAngleModel.max,
      "Mean horizontal angle must be smaller than maximum");
    GZ_ASSERT(this->currentHorzAngleModel.mu >= 0 &&
      this->currentHorzAngleModel.mu < 1,
      "Invalid process constant");
    GZ_ASSERT(this->currentHorzAngleModel.noiseAmp < 1 &&
      this->currentHorzAngleModel.noiseAmp >= 0,
      "Noise amplitude for horizontal angle has to be between 0 and 1");
  }

  this->currentHorzAngleModel.var = this->currentHorzAngleModel.mean;
  gzmsg <<
    "Current velocity horizontal angle [rad] Gauss-Markov process model:"
    << std::endl;
  this->currentHorzAngleModel.Print();

  if (currentVelocityParams->HasElement("vertical_angle"))
  {
    sdf::ElementPtr elem = currentVelocityParams->GetElement("vertical_angle");

    if (elem->HasElement("mean"))
      this->currentVertAngleModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
      this->currentVertAngleModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->currentVertAngleModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
      this->currentVertAngleModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
      this->currentVertAngleModel.noiseAmp = elem->Get<double>("noiseAmp");

    GZ_ASSERT(this->currentVertAngleModel.min <
      this->currentVertAngleModel.max, "Invalid current vertical angle limits");
    GZ_ASSERT(this->currentVertAngleModel.mean >=
      this->currentVertAngleModel.min,
      "Mean vertical angle must be greater than minimum");
    GZ_ASSERT(this->currentVertAngleModel.mean <=
      this->currentVertAngleModel.max,
      "Mean vertical angle must be smaller than maximum");
    GZ_ASSERT(this->currentVertAngleModel.mu >= 0 &&
      this->currentVertAngleModel.mu < 1,
      "Invalid process constant");
    GZ_ASSERT(this->currentVertAngleModel.noiseAmp < 1 &&
      this->currentVertAngleModel.noiseAmp >= 0,
      "Noise amplitude for vertical angle has to be between 0 and 1");
  }

  this->currentVertAngleModel.var = this->currentVertAngleModel.mean;
  gzmsg <<
    "Current velocity horizontal angle [rad] Gauss-Markov process model:"
    << std::endl;
  this->currentHorzAngleModel.Print();

  // Initialize the time update
  this->lastUpdate = this->world->GetSimTime();
  this->currentVelModel.lastUpdate = this->lastUpdate.Double();
  this->currentHorzAngleModel.lastUpdate = this->lastUpdate.Double();
  this->currentVertAngleModel.lastUpdate = this->lastUpdate.Double();

  // Advertise the current velocity topic
  this->publishers[this->currentVelocityTopic] =
    this->node->Advertise<msgs::Vector3d>(
    this->ns + "/" + this->currentVelocityTopic);

  gzmsg << "Current velocity topic name: " <<
    this->ns + "/" + this->currentVelocityTopic << std::endl;

  // Connect the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&UnderwaterCurrentPlugin::Update,
    this, _1));

  gzmsg << "Underwater current plugin loaded!" << std::endl
    << "\tWARNING: Current velocity calculated in the ENU frame"
    << std::endl;
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::Init()
{
    // Doing nothing for now
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::Update(const common::UpdateInfo & /** _info */)
{
    common::Time time = this->world->GetSimTime();
    // Calculate the flow velocity and the direction using the Gauss-Markov
    // model

    // Update current velocity
    double currentVelMag = this->currentVelModel.Update(time.Double());

    // Update current horizontal direction around z axis of flow frame
    double horzAngle = this->currentHorzAngleModel.Update(time.Double());

    // Update current horizontal direction around z axis of flow frame
    double vertAngle = this->currentVertAngleModel.Update(time.Double());

    // Generating the current velocity vector as in the NED frame
    this->currentVelocity = math::Vector3(
        currentVelMag * cos(horzAngle) * cos(vertAngle),
        currentVelMag * sin(horzAngle) * cos(vertAngle),
        currentVelMag * sin(vertAngle));

    // Update time stamp
    this->lastUpdate = time;
    this->PublishCurrentVelocity();
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::PublishCurrentVelocity()
{
    msgs::Vector3d currentVel;
    msgs::Set(&currentVel, ignition::math::Vector3d(this->currentVelocity.x,
                                                    this->currentVelocity.y,
                                                    this->currentVelocity.z));
    this->publishers[this->currentVelocityTopic]->Publish(currentVel);
}
