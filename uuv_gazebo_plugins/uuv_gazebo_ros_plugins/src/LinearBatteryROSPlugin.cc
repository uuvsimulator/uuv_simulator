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

#include <uuv_gazebo_ros_plugins/LinearBatteryROSPlugin.hh>

#include <gazebo/plugins/LinearBatteryPlugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
LinearBatteryROSPlugin::LinearBatteryROSPlugin()
{
  this->robotNamespace = "";
}

/////////////////////////////////////////////////
LinearBatteryROSPlugin::~LinearBatteryROSPlugin()
{
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::Load(physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  try
  {
    LinearBatteryPlugin::Load(_parent, _sdf);
  }
  catch(common::Exception &_e)
  {
    gzerr << "Error loading plugin."
          << "Please ensure that your model is correct."
          << '\n';
    return;
  }

  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  if (_sdf->HasElement("namespace"))
    this->robotNamespace = _sdf->Get<std::string>("namespace");

  double updateRate = 2;
  if (_sdf->HasElement("update_rate"))
    updateRate = _sdf->Get<double>("update_rate");

  if (updateRate <= 0.0)
  {
    gzmsg << "Invalid update rate, setting it to 2 Hz, rate=" << updateRate
      << std::endl;
    updateRate = 2;
  }
  this->rosNode.reset(new ros::NodeHandle(this->robotNamespace));

  this->batteryStatePub = this->rosNode->advertise<sensor_msgs::BatteryState>
    ("battery_state", 0);

  this->updateTimer = this->rosNode->createTimer(
    ros::Duration(1 / updateRate),
    boost::bind(&LinearBatteryROSPlugin::PublishBatteryState, this));

  gzmsg << "ROS Battery Plugin for link <" << this->link->GetName()
    << "> initialized\n"
    << "\t- Initial charge [Ah]=" << this->q0 << '\n'
    << "\t- Update rate [Hz]=" << updateRate
    << std::endl;
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::PublishBatteryState()
{
  this->batteryStateMsg.header.stamp = ros::Time().now();
  this->batteryStateMsg.header.frame_id = this->link->GetName();

  this->batteryStateMsg.charge = this->q;
  this->batteryStateMsg.percentage = this->q / this->q0;
  this->batteryStateMsg.voltage = this->battery->Voltage();
  this->batteryStateMsg.design_capacity = this->q0;

  this->batteryStateMsg.power_supply_status =
    sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  this->batteryStateMsg.power_supply_health =
    sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  this->batteryStateMsg.power_supply_technology =
    sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  this->batteryStateMsg.present = true;

  // Publish battery message
  this->batteryStatePub.publish(this->batteryStateMsg);
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::Init()
{
  LinearBatteryPlugin::Init();
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::Reset()
{
  LinearBatteryPlugin::Reset();
}

GZ_REGISTER_MODEL_PLUGIN(LinearBatteryROSPlugin)
}
