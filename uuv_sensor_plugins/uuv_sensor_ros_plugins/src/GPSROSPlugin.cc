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

#include <uuv_sensor_ros_plugins/GPSROSPlugin.hh>

namespace gazebo {

/////////////////////////////////////////////////
GPSROSPlugin::GPSROSPlugin() : ROSBaseSensorPlugin()
{ }

/////////////////////////////////////////////////
GPSROSPlugin::~GPSROSPlugin()
{ }

/////////////////////////////////////////////////
void GPSROSPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  gzmsg << "GPSROSPlugin - Loading base sensor plugin" << std::endl;
  ROSBaseSensorPlugin::Load(_parent, _sdf);

  gzmsg << "GPSROSPlugin - Converting GPS sensor pointer" << std::endl;
  this->gazeboGPSSensor =
    std::dynamic_pointer_cast<sensors::GpsSensor>(_parent);

  gzmsg << "GPSROSPlugin - Initialize sensor topic publisher" << std::endl;
  this->rosSensorOutputPub = this->rosNode->advertise<sensor_msgs::NavSatFix>(
    this->sensorOutputTopic, 10);

  // Set the frame ID
  this->gpsMessage.header.frame_id = this->robotNamespace + "/gps_link";
  // TODO: Get the position covariance from the GPS sensor
  this->gpsMessage.position_covariance_type =
    sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;

  double horizontalPosStdDev = 0.0;
  GetSDFParam(_sdf, "horizontal_pos_std_dev", horizontalPosStdDev, 0.0);

  double verticalPosStdDev = 0.0;
  GetSDFParam(_sdf, "vertical_pos_std_dev", verticalPosStdDev, 0.0);

  this->gpsMessage.position_covariance[0] = horizontalPosStdDev * horizontalPosStdDev;
  this->gpsMessage.position_covariance[4] = horizontalPosStdDev * horizontalPosStdDev;
  this->gpsMessage.position_covariance[8] = verticalPosStdDev * verticalPosStdDev;

  // TODO: Configurable status setup
  this->gpsMessage.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  this->gpsMessage.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  // Connect to the sensor update event.
  this->updateConnection = this->gazeboGPSSensor->ConnectUpdated(
    boost::bind(&GPSROSPlugin::OnUpdateGPS, this));
}

/////////////////////////////////////////////////
bool GPSROSPlugin::OnUpdateGPS()
{
  // Publish sensor state
  this->PublishState();
  common::Time currentTime = this->gazeboGPSSensor->LastMeasurementTime();

  this->gpsMessage.header.stamp.sec = currentTime.sec;
  this->gpsMessage.header.stamp.nsec = currentTime.nsec;

  // Copy the output of Gazebo's GPS sensor into a NavSatFix message
  this->gpsMessage.latitude = -this->gazeboGPSSensor->Latitude().Degree();
  this->gpsMessage.longitude = -this->gazeboGPSSensor->Longitude().Degree();
  this->gpsMessage.altitude = this->gazeboGPSSensor->Altitude();

  this->rosSensorOutputPub.publish(this->gpsMessage);

  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_SENSOR_PLUGIN(GPSROSPlugin)
}
