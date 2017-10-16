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

#include <uuv_sensor_plugins_ros/SwitchableROSSensorPlugin.hh>

namespace gazebo {
/////////////////////////////////////////////////
SwitchableROSSensorPlugin::SwitchableROSSensorPlugin() : SensorPlugin()
{ }

/////////////////////////////////////////////////
SwitchableROSSensorPlugin::~SwitchableROSSensorPlugin()
{ }

/////////////////////////////////////////////////
void SwitchableROSSensorPlugin::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading SwitchableROSSensorPlugin since ROS has not been "
          << "properly initialized." << std::endl;
    return;
  }

  this->robotNamespace.clear();
  if (_sdf->HasElement("namespace"))
    this->robotNamespace = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[switchable_ros_sensor_plugin] Please specify a vehicle namespace.\n";

  this->inputTopicName.clear();
  if (_sdf->HasElement("input_topic"))
    this->inputTopicName = _sdf->GetElement("input_topic")->Get<std::string>();
  else
    gzerr << "[switchable_ros_sensor_plugin] Please specify an input topic name.\n";

  bool isOn = true;
  if (_sdf->HasElement("is_on"))
    isOn = _sdf->GetElement("is_on")->Get<bool>();

  this->rosNode.reset(new ros::NodeHandle(this->robotNamespace));

#if GAZEBO_MAJOR_VERSION >= 7
  this->parentSensor = std::dynamic_pointer_cast<sensors::Sensor>(_parent);
  this->InitSwitchablePlugin(this->parentSensor->Name(), isOn);
#else
  this->parentSensor = boost::dynamic_pointer_cast<sensors::Sensor>(_parent);
  this->InitSwitchablePlugin(this->parentSensor->GetName(), isOn);
#endif

#if GAZEBO_MAJOR_VERSION >= 7
  gzmsg << this->parentSensor->Name() << ":: Switchable sensor plugin initialized" << std::endl;
#else
  gzmsg << this->parentSensor->GetName() << ":: Switchable sensor plugin initialized" << std::endl;
#endif

}

}
