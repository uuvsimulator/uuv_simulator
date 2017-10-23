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

#include <uuv_sensor_plugins_ros/GPSROSPlugin.hh>

namespace gazebo {

/////////////////////////////////////////////////
GazeboGPSROSPlugin::GazeboGPSROSPlugin() : SensorPlugin()
{

}

/////////////////////////////////////////////////
GazeboGPSROSPlugin::~GazeboGPSROSPlugin()
{
  this->gazeboGPSSensor->DisconnectUpdated(this->updateConnection);
}

/////////////////////////////////////////////////
void GazeboGPSROSPlugin::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading GazeboGPSROSPlugin since ROS has not been properly "
          << "initialized." << std::endl;
    return;
  }

  GZ_ASSERT(_sdf->HasElement("namespace"), "Robot namespace was not provided");

  this->robotNamespace = _sdf->GetElement("namespace")->Get<std::string>();
  this->rosNode.reset(new ros::NodeHandle(this->robotNamespace));

  std::string outputTopic;

  if (_sdf->HasElement("topic"))
      outputTopic = _sdf->GetElement("topic")->Get<std::string>();
  else
      gzerr << "[gazebo_gps_ros_plugin] Please specify an output topic.\n";


  this->gazeboGPSSensor =
    std::dynamic_pointer_cast<sensors::GpsSensor>(_parent);

  // Turn on Gazebo's GPS sensor
  this->gazeboGPSSensor->SetActive(true);

  this->pubGPS = this->rosNode->advertise<sensor_msgs::NavSatFix>(
    outputTopic, 10);

  // Set the frame ID
  this->gpsMessage.header.frame_id = this->robotNamespace;
  // TODO: Get the position covariance from the GPS sensor
  this->gpsMessage.position_covariance_type =
    sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;

  double horizontalPosStdDev = 0.0;
  if (_sdf->HasElement("horizontal_pos_std_dev"))
    horizontalPosStdDev =
      _sdf->GetElement("horizontal_pos_std_dev")->Get<double>();

  double verticalPosStdDev = 0.0;
  if (_sdf->HasElement("vertical_pos_std_dev"))
    verticalPosStdDev =
      _sdf->GetElement("vertical_pos_std_dev")->Get<double>();
  this->gpsMessage.position_covariance[0] = horizontalPosStdDev * horizontalPosStdDev;
  this->gpsMessage.position_covariance[4] = horizontalPosStdDev * horizontalPosStdDev;
  this->gpsMessage.position_covariance[8] = verticalPosStdDev * verticalPosStdDev;

  // TODO: Configurable status setup
  this->gpsMessage.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  this->gpsMessage.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  // TODO: Option to turn off GPS sensor while submerged

  // Connect to the sensor update event.
  this->updateConnection = this->gazeboGPSSensor->ConnectUpdated(
    boost::bind(&GazeboGPSROSPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
bool GazeboGPSROSPlugin::OnUpdate()
{
  common::Time currentTime = this->gazeboGPSSensor->LastMeasurementTime();

  this->gpsMessage.header.stamp.sec = currentTime.sec;
  this->gpsMessage.header.stamp.nsec = currentTime.nsec;

  // Copy the output of Gazebo's GPS sensor into a NavSatFix message
  this->gpsMessage.latitude = this->gazeboGPSSensor->Latitude().Degree();
  this->gpsMessage.longitude = this->gazeboGPSSensor->Longitude().Degree();
  this->gpsMessage.altitude = this->gazeboGPSSensor->Altitude();

  this->pubGPS.publish(this->gpsMessage);

  return true;
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboGPSROSPlugin);
}
