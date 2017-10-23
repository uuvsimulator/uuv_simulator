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

#include <uuv_sensor_plugins/RPTPlugin.hh>

#include <gazebo/physics/physics.hh>

#include "Common.hh"

namespace gazebo {

GazeboRPTPlugin::GazeboRPTPlugin()
  : GazeboSensorPlugin()
{
}

GazeboRPTPlugin::~GazeboRPTPlugin()
{
}

void GazeboRPTPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  GazeboSensorPlugin::Load(_model, _sdf);

  getSdfParam<double>(_sdf, "positionNoise", this->positionNoise, 0.1);

  // Prepare constant covariance part of message
  for (int i = 0 ; i < 9; i++)
  {
    if (i == 0 || i == 4 || i == 8)
    {
      this->message.add_position_covariance(this->positionNoise*
                                            this->positionNoise);
    }
    else
    {
      this->message.add_position_covariance(0.0);
    }
  }

  if (this->sensorTopic_.empty())
      this->sensorTopic_ = "rpt";
  // Advertise topic to gazebo
  this->publisher_ = this->nodeHandle_->Advertise<sensor_msgs::msgs::Rpt>(
        this->sensorTopic_, 10);
}

void GazeboRPTPlugin::SimulateMeasurement(
    const common::UpdateInfo &_info)
{
  // True position
  this->position = this->link_->GetWorldPose().pos;

  // Simulate noisy measurement
  this->position.x += this->positionNoise*this->normal_(this->rndGen_);
  this->position.y += this->positionNoise*this->normal_(this->rndGen_);
  this->position.z += this->positionNoise*this->normal_(this->rndGen_);

  this->lastMeasTime_ = _info.simTime;
  return;
}

bool GazeboRPTPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  if (!ShouldIGenerate(_info))
    return false;

  SimulateMeasurement(_info);

  // Publish simulated measurement
  gazebo::msgs::Vector3d* p = new gazebo::msgs::Vector3d();
  p->set_x(this->position.x);
  p->set_y(this->position.y);
  p->set_z(this->position.z);

  this->message.set_allocated_position(p);
  this->publisher_->Publish(this->message);

  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRPTPlugin);
}
