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

#include <uuv_gazebo_ros_plugins/ThrusterROSPlugin.hh>

#include <string>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>

namespace uuv_simulator_ros
{
/////////////////////////////////////////////////
ThrusterROSPlugin::ThrusterROSPlugin()
{
  this->rosPublishPeriod = gazebo::common::Time(0.05);
  this->lastRosPublishTime = gazebo::common::Time(0.0);
}

/////////////////////////////////////////////////
ThrusterROSPlugin::~ThrusterROSPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(
        this->rosPublishConnection);
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::SetThrustReference(
    const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr &_msg)
{
  if (std::isnan(_msg->data))
  {
    ROS_WARN("ThrusterROSPlugin: Ignoring nan command");
    return;
  }

  this->inputCommand = _msg->data;
}

/////////////////////////////////////////////////
gazebo::common::Time ThrusterROSPlugin::GetRosPublishPeriod()
{
  return this->rosPublishPeriod;
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::SetRosPublishRate(double _hz)
{
  if (_hz > 0.0)
    this->rosPublishPeriod = 1.0 / _hz;
  else
    this->rosPublishPeriod = 0.;
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::Init()
{
  ThrusterPlugin::Init();
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::Reset()
{
  this->lastRosPublishTime.Set(0, 0);
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::Load(gazebo::physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  try {
    ThrusterPlugin::Load(_parent, _sdf);
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

  this->services["set_thrust_force_efficiency"] =
    this->rosNode->advertiseService(
      this->thrusterLink->GetName() + "/set_thrust_force_efficiency",
      &ThrusterROSPlugin::SetThrustForceEfficiency, this);

  this->services["get_thrust_force_efficiency"] =
    this->rosNode->advertiseService(
      this->thrusterLink->GetName() + "/get_thrust_force_efficiency",
      &ThrusterROSPlugin::GetThrustForceEfficiency, this);

  this->services["set_dynamic_state_efficiency"] =
    this->rosNode->advertiseService(
      this->thrusterLink->GetName() + "/set_dynamic_state_efficiency",
      &ThrusterROSPlugin::SetDynamicStateEfficiency, this);

  this->services["get_dynamic_state_efficiency"] =
    this->rosNode->advertiseService(
      this->thrusterLink->GetName() + "/get_dynamic_state_efficiency",
      &ThrusterROSPlugin::GetDynamicStateEfficiency, this);

  this->services["set_thruster_state"] =
    this->rosNode->advertiseService(
      this->thrusterLink->GetName() + "/set_thruster_state",
      &ThrusterROSPlugin::SetThrusterState, this);

  this->services["get_thruster_state"] =
    this->rosNode->advertiseService(
      this->thrusterLink->GetName() + "/get_thruster_state",
      &ThrusterROSPlugin::GetThrusterState, this);

  this->subThrustReference = this->rosNode->subscribe<
    uuv_gazebo_ros_plugins_msgs::FloatStamped
    >(this->commandSubscriber->GetTopic(), 10,
      boost::bind(&ThrusterROSPlugin::SetThrustReference, this, _1));

  this->pubThrust = this->rosNode->advertise<
    uuv_gazebo_ros_plugins_msgs::FloatStamped
    >(this->thrustTopicPublisher->GetTopic(), 10);

  this->pubThrustWrench =
    this->rosNode->advertise<geometry_msgs::WrenchStamped>(
      this->thrustTopicPublisher->GetTopic() + "_wrench", 10);

  this->rosPublishConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ThrusterROSPlugin::RosPublishStates, this));
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::RosPublishStates()
{
  // Limit publish rate according to publish period
  if (this->thrustForceStamp - this->lastRosPublishTime >=
      this->rosPublishPeriod)
  {
    this->lastRosPublishTime = this->thrustForceStamp;

    uuv_gazebo_ros_plugins_msgs::FloatStamped thrust_msg;
    thrust_msg.header.stamp = ros::Time().now();
    thrust_msg.header.frame_id = this->thrusterLink->GetName();
    thrust_msg.data = this->thrustForce;
    this->pubThrust.publish(thrust_msg);

    geometry_msgs::WrenchStamped thrust_wrench_msg;
    thrust_wrench_msg.header.stamp = ros::Time().now();
    thrust_wrench_msg.header.frame_id = this->thrusterLink->GetName();

    thrust_wrench_msg.wrench.force.x = this->thrustForce;
    this->pubThrustWrench.publish(thrust_wrench_msg);
  }
}

/////////////////////////////////////////////////
bool ThrusterROSPlugin::SetThrustForceEfficiency(
  uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Response& _res)
{
  if (_req.efficiency < 0.0 || _req.efficiency > 1.0)
  {
    _res.success = false;
  }
  else
  {
    this->thrustEfficiency = _req.efficiency;
    _res.success = true;
  }
  return true;
}

/////////////////////////////////////////////////
bool ThrusterROSPlugin::GetThrustForceEfficiency(
  uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency::Response& _res)
{
  _res.efficiency = this->thrustEfficiency;
  return true;
}

/////////////////////////////////////////////////
bool ThrusterROSPlugin::SetDynamicStateEfficiency(
  uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Response& _res)
{
  if (_req.efficiency < 0.0 || _req.efficiency > 1.0)
  {
    _res.success = false;
  }
  else
  {
    this->propellerEfficiency = _req.efficiency;
    _res.success = true;
  }
  return true;
}

/////////////////////////////////////////////////
bool ThrusterROSPlugin::GetDynamicStateEfficiency(
  uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency::Response& _res)
{
  _res.efficiency = this->propellerEfficiency;
  return true;
}

/////////////////////////////////////////////////
bool ThrusterROSPlugin::SetThrusterState(
  uuv_gazebo_ros_plugins_msgs::SetThrusterState::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetThrusterState::Response& _res)
{
  this->isOn = _req.on;
  gzmsg << "Turning thruster " << this->thrusterLink->GetName() << " " <<
    (this->isOn ? "ON" : "OFF") << std::endl;
  _res.success = true;
  return true;
}

/////////////////////////////////////////////////
bool ThrusterROSPlugin::GetThrusterState(
  uuv_gazebo_ros_plugins_msgs::GetThrusterState::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetThrusterState::Response& _res)
{
  _res.is_on = this->isOn;
  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(ThrusterROSPlugin)
}
