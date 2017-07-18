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
{

}

/////////////////////////////////////////////////
SwitchableROSSensorPlugin::~SwitchableROSSensorPlugin()
{
  gzmsg << "SwitchableROSSensorPlugin - shutting down sensor" << std::endl;
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void SwitchableROSSensorPlugin::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading SwitchableROSSensorPlugin since ROS has not been properly "
          << "initialized." << std::endl;
    return;
  }

  this->robotNamespace.clear();
  if (_sdf->HasElement("namespace"))
      this->robotNamespace =
          _sdf->GetElement("namespace")->Get<std::string>();
  else
      gzerr << "[switchable_ros_sensor_plugin] Please specify a vehicle namespace.\n";

  this->inputTopicName.clear();
  if (_sdf->HasElement("input_topic"))
      this->inputTopicName =
          _sdf->GetElement("input_topic")->Get<std::string>();
  else
      gzerr << "[switchable_ros_sensor_plugin] Please specify an input topic name.\n";

  this->rosNode.reset(new ros::NodeHandle(this->robotNamespace));

#if GAZEBO_MAJOR_VERSION >= 7
  this->parentSensor = std::dynamic_pointer_cast<sensors::Sensor>(_parent);
#else
  this->parentSensor = boost::dynamic_pointer_cast<sensors::Sensor>(_parent);
#endif  

  this->sensorOn = false;

  this->changeSensorSrv = this->rosNode->advertiseService(
    this->parentSensor->Name() + "/change_sensor_state",
    &SwitchableROSSensorPlugin::ChangeSensorState, this);

#if GAZEBO_MAJOR_VERSION >= 7
  gzmsg << this->parentSensor->Name() << ":: Switchable sensor plugin initialized" << std::endl;
#else
  gzmsg << this->parentSensor->GetName() << ":: Switchable sensor plugin initialized" << std::endl;
#endif
}

/////////////////////////////////////////////////
bool SwitchableROSSensorPlugin::ChangeSensorState(
    uuv_sensor_plugins_ros_msgs::ChangeSensorState::Request& _req,
    uuv_sensor_plugins_ros_msgs::ChangeSensorState::Response& _res)
{
  this->sensorOn = _req.on;
  _res.success = true;
  std::string message = "";

#if GAZEBO_MAJOR_VERSION >= 7
  message += this->parentSensor->ParentName() + "::" + this->parentSensor->Name();
#else
  message += this->parentSensor->GetParentName() + "::" + this->parentSensor->GetName();
#endif

  if (_req.on)
    message += " ON";
  else
    message += " OFF";
  _res.message = message;
  gzmsg << message << std::endl;
  return true;
}

}