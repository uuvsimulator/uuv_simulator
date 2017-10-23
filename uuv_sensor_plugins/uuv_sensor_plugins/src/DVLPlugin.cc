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

#include <uuv_sensor_plugins/DVLPlugin.hh>

#include <gazebo/physics/physics.hh>

#include "SensorDvl.pb.h"
#include "Common.hh"

namespace gazebo {

GazeboDVLPlugin::GazeboDVLPlugin()
  : GazeboSensorPlugin()
{
}

GazeboDVLPlugin::~GazeboDVLPlugin()
{
}

void GazeboDVLPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  GazeboSensorPlugin::Load(_model, _sdf);

  getSdfParam<double>(_sdf, "velocityNoise", this->velocityNoise, 0.1);

  // Prepare constant covariance part of message
  for (int i = 0 ; i < 9; i++)
  {
    if (i == 0 || i == 4 || i == 8)
    {
      this->message.add_linear_velocity_covariance(this->velocityNoise*
                                                   this->velocityNoise);
    }
    else
    {
      this->message.add_linear_velocity_covariance(0.0);
    }
  }
  if (this->sensorTopic_.empty())
    this->sensorTopic_ = "dvl";

  // Advertise topic to gazebo
  this->publisher_ = this->nodeHandle_->Advertise<sensor_msgs::msgs::Dvl>(
        this->sensorTopic_, 10);
}

void GazeboDVLPlugin::SimulateMeasurement(
    const common::UpdateInfo &_info)
{
  // True velocity
  this->vel = this->link_->GetRelativeLinearVel();

  // Simulate noisy measurement
  this->vel.x += this->velocityNoise*this->normal_(this->rndGen_);
  this->vel.y += this->velocityNoise*this->normal_(this->rndGen_);
  this->vel.z += this->velocityNoise*this->normal_(this->rndGen_);

  this->lastMeasTime_ = _info.simTime;
  return;
}

bool GazeboDVLPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  if (!ShouldIGenerate(_info))
    return false;

  SimulateMeasurement(_info);

  // Publish simulated measurement
  gazebo::msgs::Vector3d* v = new gazebo::msgs::Vector3d();
  v->set_x(this->vel.x);
  v->set_y(this->vel.y);
  v->set_z(this->vel.z);

  this->message.set_allocated_linear_velocity(v);
  this->publisher_->Publish(this->message);

  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboDVLPlugin);
}
