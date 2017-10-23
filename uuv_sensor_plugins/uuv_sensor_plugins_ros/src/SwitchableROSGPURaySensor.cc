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

#include <uuv_sensor_plugins_ros/SwitchableROSGPURaySensor.hh>

namespace gazebo {

/////////////////////////////////////////////////
SwitchableROSGPURaySensor::SwitchableROSGPURaySensor() : SwitchableROSSensorPlugin()
{ }

/////////////////////////////////////////////////
SwitchableROSGPURaySensor::~SwitchableROSGPURaySensor()
{ }

/////////////////////////////////////////////////
void SwitchableROSGPURaySensor::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  SwitchableROSSensorPlugin::Load(_parent, _sdf);

  this->inputSub = this->rosNode->subscribe<sensor_msgs::LaserScan>(
    this->inputTopicName, 10,
    boost::bind(&SwitchableROSGPURaySensor::UpdateSensorInput,
    this, _1));

  this->outputPub = this->rosNode->advertise<sensor_msgs::LaserScan>(
    this->inputTopicName + UUV_SWITCHABLE_SUFFIX, 1);
}

/////////////////////////////////////////////////
void SwitchableROSGPURaySensor::UpdateSensorInput(
  const sensor_msgs::LaserScan::ConstPtr &_msg)
{
  // Publish sensor state
  this->PublishState();

  if (!this->IsOn())
    return;
  this->msg.header = _msg->header;
  this->msg.angle_min = _msg->angle_min;
  this->msg.angle_max = _msg->angle_max;
  this->msg.angle_increment = _msg->angle_increment;
  this->msg.time_increment = _msg->time_increment;
  this->msg.range_min = _msg->range_min;
  this->msg.range_max = _msg->range_max;
  this->msg.ranges = _msg->ranges;
  this->msg.intensities = _msg->intensities;
  this->outputPub.publish(this->msg);
}

GZ_REGISTER_SENSOR_PLUGIN(SwitchableROSGPURaySensor)

}
