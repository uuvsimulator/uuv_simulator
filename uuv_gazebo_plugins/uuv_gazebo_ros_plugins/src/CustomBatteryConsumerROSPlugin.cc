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

#include <uuv_gazebo_ros_plugins/CustomBatteryConsumerROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
CustomBatteryConsumerROSPlugin::CustomBatteryConsumerROSPlugin()
{
  this->isDeviceOn = true;
}

/////////////////////////////////////////////////
CustomBatteryConsumerROSPlugin::~CustomBatteryConsumerROSPlugin()
{
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::Load(physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->rosNode.reset(new ros::NodeHandle(""));

  GZ_ASSERT(_sdf->HasElement("link_name"), "Consumer link name is missing");
  this->linkName = _sdf->Get<std::string>("link_name");
  physics::LinkPtr link = _parent->GetLink(this->linkName);
  GZ_ASSERT(link, "Link was NULL");

  GZ_ASSERT(_sdf->HasElement("battery_name"), "Battery name is missing");
  this->batteryName = _sdf->Get<std::string>("battery_name");
  this->battery = link->Battery(this->batteryName);
  GZ_ASSERT(this->battery, "Battery was NULL");

  GZ_ASSERT(_sdf->HasElement("power_load"), "Power load is missing");
  this->powerLoad = _sdf->Get<double>("power_load");

  GZ_ASSERT(this->powerLoad > 0, "Power load must be greater than zero");

  // Adding consumer
  this->consumerID = this->battery->AddConsumer();

  if (_sdf->HasElement("topic_device_state"))
  {
    std::string topicName = _sdf->Get<std::string>("topic_device_state");
    GZ_ASSERT(!topicName.empty(), "Topic name is empty");
    this->deviceStateSub = this->rosNode->subscribe<std_msgs::Bool>(
      topicName, 1,
      boost::bind(&CustomBatteryConsumerROSPlugin::UpdateDeviceState, this, _1));
  }
  else
  {
    // In the case the device is always on, then set the power load only once
    this->UpdatePowerLoad(this->powerLoad);
  }

  gzmsg << "CustomBatteryConsumerROSPlugin::Device <"
    << this->linkName << "> added as battery consumer" << std::endl
    << "\t- ID=" << this->consumerID << std::endl
    << "\t- Power load [W]=" << this->powerLoad
    << std::endl;
}

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::UpdateDeviceState(
  const std_msgs::Bool::ConstPtr &_msg)
{
  this->isDeviceOn = _msg->data;
  if (this->isDeviceOn)
    this->UpdatePowerLoad(this->powerLoad);
  else
    this->UpdatePowerLoad(0.0); 
}

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::UpdatePowerLoad(double _powerLoad)
{
  if (!this->battery->SetPowerLoad(this->consumerID, _powerLoad))
    gzerr << "Error setting the consumer power load" << std::endl;
}

GZ_REGISTER_MODEL_PLUGIN(CustomBatteryConsumerROSPlugin)
}
