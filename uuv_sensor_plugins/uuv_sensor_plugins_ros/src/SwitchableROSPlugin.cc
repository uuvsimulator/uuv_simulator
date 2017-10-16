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

#include <uuv_sensor_plugins_ros/SwitchableROSPlugin.hh>

namespace gazebo {
/////////////////////////////////////////////////
SwitchableROSPlugin::SwitchableROSPlugin()
{
  this->topicNamePrefix = "";
  this->isOn.data = true;
}

/////////////////////////////////////////////////
SwitchableROSPlugin::~SwitchableROSPlugin()
{
  gzmsg << "SwitchableROSPlugin <" << this->topicNamePrefix <<
    "> shutting down" << std::endl;
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
bool SwitchableROSPlugin::InitSwitchablePlugin(
  std::string _topicPrefix, bool _isOn)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading SwitchableROSPlugin since ROS has not been properly "
          << "initialized." << std::endl;
    return false;
  }

  GZ_ASSERT(!parentName.empty(), "Parent name is empty");
  this->topicNamePrefix = _topicPrefix;
  this->isOn.data = _isOn;

  // The ROS node is expected to be initialized under a the namespace of the
  // plugin running this module
  this->changeSensorSrv = this->rosNode->advertiseService(
    this->topicNamePrefix + "/change_state",
    &SwitchableROSPlugin::ChangeSensorState, this);

  this->pluginStatePub = this->rosNode->advertise<std_msgs::Bool>(
    this->topicNamePrefix + "/state", 1);
}

/////////////////////////////////////////////////
bool SwitchableROSPlugin::IsOn()
{
  return this->isOn.data;
}
/////////////////////////////////////////////////
void SwitchableROSPlugin::PublishState()
{
  this->pluginStatePub.publish(this->isOn);
}

/////////////////////////////////////////////////
bool SwitchableROSPlugin::ChangeSensorState(
    uuv_sensor_plugins_ros_msgs::ChangeSensorState::Request& _req,
    uuv_sensor_plugins_ros_msgs::ChangeSensorState::Response& _res)
{
  this->isOn.data = _req.on;
  _res.success = true;
  std::string message = this->topicNamePrefix + "::";

  if (_req.on)
    message += " ON";
  else
    message += " OFF";
  _res.message = message;
  gzmsg << message << std::endl;
  return true;
}

}
