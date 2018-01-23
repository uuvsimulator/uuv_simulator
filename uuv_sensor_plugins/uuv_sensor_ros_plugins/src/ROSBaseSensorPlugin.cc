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

#include <uuv_sensor_ros_plugins/ROSBaseSensorPlugin.hh>

namespace gazebo
{

/////////////////////////////////////////////////
ROSBaseSensorPlugin::ROSBaseSensorPlugin()
{ }

/////////////////////////////////////////////////
ROSBaseSensorPlugin::~ROSBaseSensorPlugin()
{ }

/////////////////////////////////////////////////
void ROSBaseSensorPlugin::Load(sensors::SensorPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize model pointer
  #if GAZEBO_MAJOR_VERSION >= 7
    this->parentSensor = std::dynamic_pointer_cast<sensors::Sensor>(_model);
  #else
    this->parentSensor = boost::dynamic_pointer_cast<sensors::Sensor>(_model);
  #endif

  // Get the world name.
  std::string worldName = _model->WorldName();
  this->world = physics::get_world(worldName);

  this->InitBasePlugin(_sdf);
}

/////////////////////////////////////////////////
bool ROSBaseSensorPlugin::OnUpdate(const common::UpdateInfo&)
{
  return true;
}

}
