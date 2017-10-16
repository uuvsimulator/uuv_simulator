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

#include <uuv_sensor_plugins/SensorPlugin.hh>

#include <string>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include "Common.hh"

namespace gazebo {

GazeboSensorPlugin::GazeboSensorPlugin() : ModelPlugin()
{ }

GazeboSensorPlugin::~GazeboSensorPlugin()
{
    if (this->updateConnection_)
    {
        event::Events::DisconnectWorldUpdateBegin(this->updateConnection_);
    }
}

void GazeboSensorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store pointers to model, world
    this->model_ = _model;
    this->world_ = model_->GetWorld();

    // default params
    this->namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
        this->namespace_ =
            _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_sensor_plugin] Please specify a robotNamespace.\n";

    this->nodeHandle_ = transport::NodePtr(new transport::Node());
    this->nodeHandle_->Init(this->namespace_);

    if (_sdf->HasElement("linkName"))
        this->linkName_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
        gzerr << "[gazebo_sensor_plugin] Please specify a linkName.\n";

    // Get the pointer to the link
    this->link_ = this->model_->GetLink(this->linkName_);
    if (this->link_ == NULL)
    {
        gzthrow("[gazebo_sensor_plugin] Could not find specified link \""
                << this->linkName_ << "\".");
    }

    if (_sdf->HasElement("sensorTopic"))
        this->sensorTopic_ =
            _sdf->GetElement("sensorTopic")->Get<std::string>();
    else
        this->sensorTopic_ = std::string();

    if (!_sdf->HasElement("updatePeriod"))
        gzerr << "[gazebo_sensor_plugin] Please specify an updatePeriod.\n";

    double period;
    getSdfParam<double>(_sdf, "updatePeriod", period, 1./10);
    this->updatePeriod_.Set(period);

    this->lastMeasTime_ = this->world_->GetSimTime();

    this->normal_ = std::normal_distribution<double>(0.0, 1.0);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ =
            event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboSensorPlugin::OnUpdate, this, _1));
}

bool GazeboSensorPlugin::ShouldIGenerate(const common::UpdateInfo& _info) const
{
    common::Time current_time  = _info.simTime;
    double dt = (current_time - this->lastMeasTime_).Double();
    return dt >= this->updatePeriod_.Double();
}
}
