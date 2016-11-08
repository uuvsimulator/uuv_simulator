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

/// \file UnderwaterWorldPlugin.cc

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

#include "uuv_world_plugins/UnderwaterWorldPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(UnderwaterWorldPlugin)

/////////////////////////////////////////////////
UnderwaterWorldPlugin::UnderwaterWorldPlugin()
{
  // Doing nothing for now
}

/////////////////////////////////////////////////
UnderwaterWorldPlugin::~UnderwaterWorldPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void UnderwaterWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
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
  GZ_ASSERT(currentVelocityParams->HasElement("topic"),
    "Current velocity topic name is not available");

  // Retrieve the current velocity topic
  this->currentVelocityTopic =
      currentVelocityParams->Get<std::string>("topic");

  gzmsg << "current velocity " << this->currentVelocityTopic << std::endl;

  GZ_ASSERT(!this->currentVelocityTopic.empty(),
    "Empty current velocity topic");

  if (this->currentVelocityTopic[0] != '/')
    this->currentVelocityTopic = "/" + this->currentVelocityTopic;

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

  if (_sdf->HasElement("direction"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("direction");

    if (elem->HasElement("mean"))
      this->currentDirectionModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
      this->currentDirectionModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->currentDirectionModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
      this->currentDirectionModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
      this->currentVelModel.noiseAmp = elem->Get<double>("noiseAmp");

    GZ_ASSERT(this->currentDirectionModel.min <
      this->currentDirectionModel.max, "Invalid current direction limits");
    GZ_ASSERT(this->currentDirectionModel.mean >=
      this->currentDirectionModel.min,
      "Mean direction must be greater than minimum");
    GZ_ASSERT(this->currentDirectionModel.mean <=
      this->currentDirectionModel.max,
      "Mean direction must be smaller than maximum");
    GZ_ASSERT(this->currentDirectionModel.mu >= 0 &&
      this->currentDirectionModel.mu < 1,
      "Invalid process constant");
    GZ_ASSERT(this->currentDirectionModel.noiseAmp < 1 &&
      this->currentDirectionModel.noiseAmp >= 0,
      "Noise amplitude has to be smaller than 1");
  }

  this->currentDirectionModel.var = this->currentDirectionModel.mean;

  // Initialize the time update
  this->lastUpdate = this->world->GetSimTime();
  this->currentVelModel.lastUpdate = this->lastUpdate.Double();
  this->currentDirectionModel.lastUpdate = this->lastUpdate.Double();

  // Advertise the current velocity topic
  this->publishers[this->currentVelocityTopic] =
    this->node->Advertise<msgs::Vector3d>(
    this->ns + this->currentVelocityTopic);

  gzmsg << this->ns + this->currentVelocityTopic << std::endl;

  // Connect the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&UnderwaterWorldPlugin::Update,
    this, _1));

  gzmsg << "Underwater world loaded!" << std::endl;
}

/////////////////////////////////////////////////
void UnderwaterWorldPlugin::Init()
{
    // Doing nothing for now
}

/////////////////////////////////////////////////
void UnderwaterWorldPlugin::Update(const common::UpdateInfo & /** _info */)
{
    common::Time time = this->world->GetSimTime();
    // Calculate the flow velocity and the direction using the Gauss-Markov
    // model

    // Update current velocity
    double currentVelMag = this->currentVelModel.Update(time.Double());

    // Update current horizontal direction around z axis of flow frame
    double horizontalDir = this->currentDirectionModel.Update(time.Double());

    this->currentVelocity = math::Vector3(currentVelMag * cos(horizontalDir),
                                          currentVelMag * sin(horizontalDir),
                                          0);

    // Update time stamp
    this->lastUpdate = time;
    this->PublishCurrentVelocity();
}

/////////////////////////////////////////////////
void UnderwaterWorldPlugin::PublishCurrentVelocity()
{
    msgs::Vector3d currentVel;
    msgs::Set(&currentVel, ignition::math::Vector3d(this->currentVelocity.x,
                                                   this->currentVelocity.y,
                                                   this->currentVelocity.z));
    this->publishers[this->currentVelocityTopic]->Publish(currentVel);
}
