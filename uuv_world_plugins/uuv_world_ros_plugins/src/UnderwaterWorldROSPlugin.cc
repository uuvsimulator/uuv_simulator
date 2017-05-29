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

#include <uuv_world_ros_plugins/UnderwaterWorldROSPlugin.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>

namespace uuv_simulator_ros
{
/////////////////////////////////////////////////
UnderwaterWorldROSPlugin::UnderwaterWorldROSPlugin()
{
  this->rosPublishPeriod = gazebo::common::Time(0.05);
  this->lastRosPublishTime = gazebo::common::Time(0.0);
}

/////////////////////////////////////////////////
UnderwaterWorldROSPlugin::~UnderwaterWorldROSPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(
    this->rosPublishConnection);
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void UnderwaterWorldROSPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  try
  {
    UnderwaterWorldPlugin::Load(_world, _sdf);
  } catch(gazebo::common::Exception &_e)
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

  this->rosNode.reset(new ros::NodeHandle(""));

  // Advertise the flow velocity as a stamped twist message
  this->flowVelocityPub = this->rosNode->advertise<geometry_msgs::TwistStamped>(
    this->publishers[this->currentVelocityTopic]->GetTopic(), 10);

  // Advertise the service to update the current velocity model
  this->worldServices["set_current_velocity_model"] =
    this->rosNode->advertiseService(
    "/" + this->ns + "/set_current_velocity_model",
    &UnderwaterWorldROSPlugin::UpdateCurrentVelocityModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["get_current_velocity_model"] =
    this->rosNode->advertiseService(
    "/" + this->ns + "/get_current_velocity_model",
    &UnderwaterWorldROSPlugin::GetCurrentVelocityModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["set_current_horz_angle_model"] =
    this->rosNode->advertiseService(
    "/" + this->ns + "/set_current_horz_angle_model",
    &UnderwaterWorldROSPlugin::UpdateCurrentHorzAngleModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["get_current_horz_angle_model"] =
    this->rosNode->advertiseService(
    "/" + this->ns + "/get_current_horz_angle_model",
    &UnderwaterWorldROSPlugin::GetCurrentHorzAngleModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["set_current_vert_angle_model"] =
    this->rosNode->advertiseService(
    "/" + this->ns + "/set_current_vert_angle_model",
    &UnderwaterWorldROSPlugin::UpdateCurrentVertAngleModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["get_current_vert_angle_model"] =
    this->rosNode->advertiseService(
    "/" + this->ns + "/get_current_vert_angle_model",
    &UnderwaterWorldROSPlugin::GetCurrentHorzAngleModel, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_current_velocity"] =
    this->rosNode->advertiseService(
      "/" + this->ns + "/set_current_velocity",
      &UnderwaterWorldROSPlugin::UpdateCurrentVelocity, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_current_horz_angle"] =
    this->rosNode->advertiseService(
      "/" + this->ns + "/set_current_horz_angle",
      &UnderwaterWorldROSPlugin::UpdateHorzAngle, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_current_vert_angle"] =
    this->rosNode->advertiseService(
      "/" + this->ns + "/set_current_vert_angle",
      &UnderwaterWorldROSPlugin::UpdateVertAngle, this);

  this->rosPublishConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&UnderwaterWorldROSPlugin::PublishROSTopics, this));
}

/////////////////////////////////////////////////
void UnderwaterWorldROSPlugin::PublishROSTopics()
{
  if (this->lastUpdate - this->lastRosPublishTime >= this->rosPublishPeriod)
  {
    this->lastRosPublishTime = this->lastUpdate;
    geometry_msgs::TwistStamped flowVelMsg;
    flowVelMsg.header.stamp = ros::Time().now();
    flowVelMsg.header.frame_id = "/world";

    flowVelMsg.twist.linear.x = this->currentVelocity.x;
    flowVelMsg.twist.linear.y = this->currentVelocity.y;
    flowVelMsg.twist.linear.z = this->currentVelocity.z;

    this->flowVelocityPub.publish(flowVelMsg);
  }
}

/////////////////////////////////////////////////
bool UnderwaterWorldROSPlugin::UpdateHorzAngle(
    uuv_world_ros_plugins_msgs::SetCurrentDirection::Request& _req,
    uuv_world_ros_plugins_msgs::SetCurrentDirection::Response& _res)
{
  _res.success = this->currentHorzAngleModel.SetMean(_req.angle);

  return true;
}

/////////////////////////////////////////////////
bool UnderwaterWorldROSPlugin::UpdateVertAngle(
    uuv_world_ros_plugins_msgs::SetCurrentDirection::Request& _req,
    uuv_world_ros_plugins_msgs::SetCurrentDirection::Response& _res)
{
  _res.success = this->currentVertAngleModel.SetMean(_req.angle);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterWorldROSPlugin::UpdateCurrentVelocity(
    uuv_world_ros_plugins_msgs::SetCurrentVelocity::Request& _req,
    uuv_world_ros_plugins_msgs::SetCurrentVelocity::Response& _res)
{
  if (this->currentVelModel.SetMean(_req.velocity) &&
      this->currentHorzAngleModel.SetMean(_req.horizontal_angle) &&
      this->currentVertAngleModel.SetMean(_req.vertical_angle))
  {
    gzmsg << "Current velocity [m/s] = " << _req.velocity << std::endl;
    gzmsg << "Current horizontal angle [rad] = " << _req.horizontal_angle
        << std::endl;
    gzmsg << "Current vertical angle [rad] = " << _req.vertical_angle
        << std::endl;
    gzmsg << "\tWARNING: Current velocity calculated in the ENU frame"
        << std::endl;
    _res.success = true;
  }
  else
  {
    gzmsg << "Error while updating the current velocity" << std::endl;
    _res.success = false;
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterWorldROSPlugin::GetCurrentVelocityModel(
    uuv_world_ros_plugins_msgs::GetCurrentModel::Request& _req,
    uuv_world_ros_plugins_msgs::GetCurrentModel::Response& _res)
{
  _res.mean = this->currentVelModel.mean;
  _res.min = this->currentVelModel.min;
  _res.max = this->currentVelModel.max;
  _res.noise = this->currentVelModel.noiseAmp;
  _res.mu = this->currentVelModel.mu;
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterWorldROSPlugin::GetCurrentHorzAngleModel(
    uuv_world_ros_plugins_msgs::GetCurrentModel::Request& _req,
    uuv_world_ros_plugins_msgs::GetCurrentModel::Response& _res)
{
  _res.mean = this->currentHorzAngleModel.mean;
  _res.min = this->currentHorzAngleModel.min;
  _res.max = this->currentHorzAngleModel.max;
  _res.noise = this->currentHorzAngleModel.noiseAmp;
  _res.mu = this->currentHorzAngleModel.mu;
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterWorldROSPlugin::GetCurrentVertAngleModel(
    uuv_world_ros_plugins_msgs::GetCurrentModel::Request& _req,
    uuv_world_ros_plugins_msgs::GetCurrentModel::Response& _res)
{
  _res.mean = this->currentVertAngleModel.mean;
  _res.min = this->currentVertAngleModel.min;
  _res.max = this->currentVertAngleModel.max;
  _res.noise = this->currentVertAngleModel.noiseAmp;
  _res.mu = this->currentVertAngleModel.mu;
  return true;
}


/////////////////////////////////////////////////
bool UnderwaterWorldROSPlugin::UpdateCurrentVelocityModel(
    uuv_world_ros_plugins_msgs::SetCurrentModel::Request& _req,
    uuv_world_ros_plugins_msgs::SetCurrentModel::Response& _res)
{
  _res.success = this->currentVelModel.SetModel(_req.mean, _req.min, _req.max,
    _req.mu, _req.noise);
  gzmsg << "Current velocity model updated" << std::endl;
  gzmsg << "\tWARNING: Current velocity calculated in the ENU frame"
      << std::endl;
  this->currentVelModel.Print();
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterWorldROSPlugin::UpdateCurrentHorzAngleModel(
    uuv_world_ros_plugins_msgs::SetCurrentModel::Request& _req,
    uuv_world_ros_plugins_msgs::SetCurrentModel::Response& _res)
{
  _res.success = this->currentHorzAngleModel.SetModel(_req.mean, _req.min,
    _req.max, _req.mu, _req.noise);
  gzmsg << "Horizontal angle model updated" << std::endl;
  gzmsg << "\tWARNING: Current velocity calculated in the ENU frame"
      << std::endl;
  this->currentHorzAngleModel.Print();
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterWorldROSPlugin::UpdateCurrentVertAngleModel(
    uuv_world_ros_plugins_msgs::SetCurrentModel::Request& _req,
    uuv_world_ros_plugins_msgs::SetCurrentModel::Response& _res)
{
  _res.success = this->currentVertAngleModel.SetModel(_req.mean, _req.min,
    _req.max, _req.mu, _req.noise);
  gzmsg << "Vertical angle model updated" << std::endl;
  gzmsg << "\tWARNING: Current velocity calculated in the ENU frame"
      << std::endl;
  this->currentVertAngleModel.Print();
  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_WORLD_PLUGIN(UnderwaterWorldROSPlugin)
}
