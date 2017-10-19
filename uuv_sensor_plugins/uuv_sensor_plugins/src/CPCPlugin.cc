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

#include <uuv_sensor_plugins/CPCPlugin.hh>

namespace gazebo
{

/////////////////////////////////////////////////
GazeboCPCPlugin::GazeboCPCPlugin()
  : GazeboSensorPlugin()
{ }

/////////////////////////////////////////////////
GazeboCPCPlugin::~GazeboCPCPlugin()
{ }

/////////////////////////////////////////////////
void GazeboCPCPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GazeboSensorPlugin::Load(_model, _sdf);

  if (this->sensorTopic_.empty())
    this->sensorTopic_ = "particle_concentration";
}

/////////////////////////////////////////////////
bool GazeboCPCPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  if (!this->ShouldIGenerate(_info))
    return false;

  // Do nothing for now

  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(GazeboCPCPlugin)
}
