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

#include <uuv_sensor_plugins_ros/AcousticSonarROSPlugin.hh>

namespace gazebo {

/////////////////////////////////////////////////
GazeboAcousticSonarRosPlugin::GazeboAcousticSonarRosPlugin() : SensorPlugin()
{

}

/////////////////////////////////////////////////
GazeboAcousticSonarRosPlugin::~GazeboAcousticSonarRosPlugin()
{
  this->gazeboSensor->DisconnectUpdated(this->updateConnection);
}

/////////////////////////////////////////////////
void GazeboAcousticSonarRosPlugin::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading GazeboAcousticSonarRosPlugin since ROS has not been properly "
          << "initialized." << std::endl;
    return;
  }

  this->vehicleNamespace.clear();
  if (_sdf->HasElement("namespace"))
      this->vehicleNamespace =
          _sdf->GetElement("namespace")->Get<std::string>();
  else
      gzerr << "[gazebo_acoustic_sonar_ros_plugin] Please specify a vehicle namespace.\n";

  this->inputTopicName.clear();
  if (_sdf->HasElement("input_topic"))
      this->inputTopicName =
          _sdf->GetElement("input_topic")->Get<std::string>();
  else
      gzerr << "[gazebo_acoustic_sonar_ros_plugin] Please specify an input topic name.\n";

  this->rosNode.reset(new ros::NodeHandle(this->vehicleNamespace));

  this->gazeboSensor = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);

  // Turn on Gazebo's GPS sensor
  this->gazeboSensor->SetActive(false);

  this->sensorOn = false;

  this->laserScanInputSub = this->rosNode->subscribe<sensor_msgs::LaserScan>(
    this->inputTopicName, 10,
    boost::bind(&GazeboAcousticSonarRosPlugin::UpdateSensorInput,
    this, _1));

  this->laserScanInputPub = this->rosNode->advertise<sensor_msgs::LaserScan>(
    this->inputTopicName + "_switchable", 1);

  this->changeSensorSrv = this->rosNode->advertiseService(
    this->gazeboSensor->Name() + "/change_sensor_state",
    &GazeboAcousticSonarRosPlugin::ChangeSensorState, this);

  gzmsg << this->gazeboSensor->Name() << "::AcousticSonarPlugin initialized" << std::endl;
}

/////////////////////////////////////////////////
void GazeboAcousticSonarRosPlugin::UpdateSensorInput(
  const sensor_msgs::LaserScan::ConstPtr &_msg)
{
  this->msg.header = _msg->header;
  this->msg.angle_min = _msg->angle_min;
  this->msg.angle_max = _msg->angle_max;
  this->msg.angle_increment = _msg->angle_increment;
  this->msg.time_increment = _msg->time_increment;
  this->msg.range_min = _msg->range_min;
  this->msg.range_max = _msg->range_max;
  this->msg.ranges = _msg->ranges;
  this->msg.intensities = _msg->intensities;
  if (this->sensorOn)
    this->laserScanInputPub.publish(this->msg);
}

/////////////////////////////////////////////////
bool GazeboAcousticSonarRosPlugin::ChangeSensorState(
    uuv_sensor_plugins_ros_msgs::ChangeSensorState::Request& _req,
    uuv_sensor_plugins_ros_msgs::ChangeSensorState::Response& _res)
{
  this->sensorOn = _req.on;
  _res.success = true;
  std::string message = this->gazeboSensor->Name() + " - Sonar ";
  if (_req.on)
    message += "ON";
  else
    message += "OFF";
  _res.message = message;
  gzmsg << message << std::endl;
  return true;
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboAcousticSonarRosPlugin)

}
