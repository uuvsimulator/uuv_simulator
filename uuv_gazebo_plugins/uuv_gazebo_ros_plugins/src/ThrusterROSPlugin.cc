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
#if GAZEBO_MAJOR_VERSION >= 8
  this->rosPublishConnection.reset();
#else
  gazebo::event::Events::DisconnectWorldUpdateBegin(
    this->rosPublishConnection);
#endif

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
      this->topicPrefix + "set_thrust_force_efficiency",
      &ThrusterROSPlugin::SetThrustForceEfficiency, this);

  this->services["get_thrust_force_efficiency"] =
    this->rosNode->advertiseService(
      this->topicPrefix + "get_thrust_force_efficiency",
      &ThrusterROSPlugin::GetThrustForceEfficiency, this);

  this->services["set_dynamic_state_efficiency"] =
    this->rosNode->advertiseService(
      this->topicPrefix + "set_dynamic_state_efficiency",
      &ThrusterROSPlugin::SetDynamicStateEfficiency, this);

  this->services["get_dynamic_state_efficiency"] =
    this->rosNode->advertiseService(
      this->topicPrefix + "get_dynamic_state_efficiency",
      &ThrusterROSPlugin::GetDynamicStateEfficiency, this);

  this->services["set_thruster_state"] =
    this->rosNode->advertiseService(
      this->topicPrefix + "set_thruster_state",
      &ThrusterROSPlugin::SetThrusterState, this);

  this->services["get_thruster_state"] =
    this->rosNode->advertiseService(
      this->topicPrefix + "get_thruster_state",
      &ThrusterROSPlugin::GetThrusterState, this);

  this->services["get_thruster_conversion_fcn"] =
    this->rosNode->advertiseService(
      this->topicPrefix + "get_thruster_conversion_fcn",
      &ThrusterROSPlugin::GetThrusterConversionFcn, this);

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

  this->pubThrusterState = this->rosNode->advertise<std_msgs::Bool>(
    this->topicPrefix + "is_on", 1);

  this->pubThrustForceEff = this->rosNode->advertise<std_msgs::Float64>(
    this->topicPrefix + "thrust_efficiency", 1);

  this->pubDynamicStateEff = this->rosNode->advertise<std_msgs::Float64>(
    this->topicPrefix + "dynamic_state_efficiency", 1);

  gzmsg << "Thruster #" << this->thrusterID << " initialized" << std::endl
    << "\t- Link: " << this->thrusterLink->GetName() << std::endl
    << "\t- Robot model: " << _parent->GetName() << std::endl
    << "\t- Input command topic: " <<
      this->commandSubscriber->GetTopic() << std::endl
    << "\t- Thrust output topic: " <<
      this->thrustTopicPublisher->GetTopic() << std::endl;

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

    // Publish the thrust force magnitude
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrustMsg;
    thrustMsg.header.stamp = ros::Time().now();
    thrustMsg.header.frame_id = this->thrusterLink->GetName();
    thrustMsg.data = this->thrustForce;
    this->pubThrust.publish(thrustMsg);

    // Publish the thrust force vector wrt the thruster frame
    geometry_msgs::WrenchStamped thrustWrenchMsg;
    thrustWrenchMsg.header.stamp = ros::Time().now();
    thrustWrenchMsg.header.frame_id = this->thrusterLink->GetName();
    ignition::math::Vector3d thrustVector = this->thrustForce * this->thrusterAxis;
    thrustWrenchMsg.wrench.force.x = thrustVector.X();
    thrustWrenchMsg.wrench.force.y = thrustVector.Y();
    thrustWrenchMsg.wrench.force.z = thrustVector.Z();
    this->pubThrustWrench.publish(thrustWrenchMsg);

    // Publish the thruster current state (ON or OFF)
    std_msgs::Bool isOnMsg;
    isOnMsg.data = this->isOn;
    this->pubThrusterState.publish(isOnMsg);

    // Publish thrust output efficiency
    std_msgs::Float64 thrustEffMsg;
    thrustEffMsg.data = this->thrustEfficiency;
    this->pubThrustForceEff.publish(thrustEffMsg);

    // Publish dynamic state efficiency
    std_msgs::Float64 dynStateEffMsg;
    dynStateEffMsg.data = this->propellerEfficiency;
    this->pubDynamicStateEff.publish(dynStateEffMsg);
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
    gzmsg << "Setting thrust efficiency at thruster " <<
      this->thrusterLink->GetName() << "=" << _req.efficiency  * 100
      << "%" << std::endl;
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
    gzmsg << "Setting propeller efficiency at thruster " <<
      this->thrusterLink->GetName() << "=" << _req.efficiency * 100
      << "%" << std::endl;
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
bool ThrusterROSPlugin::GetThrusterConversionFcn(
  uuv_gazebo_ros_plugins_msgs::GetThrusterConversionFcn::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetThrusterConversionFcn::Response& _res)
{
  _res.fcn.function_name = this->conversionFunction->GetType();

  double param;

  if (!_res.fcn.function_name.compare("Basic"))
  {
    gzmsg << "ThrusterROSPlugin::GetThrusterConversionFcn::Basic" << std::endl;
    _res.fcn.tags.push_back("rotor_constant");
    this->conversionFunction->GetParam("rotor_constant", param);
    _res.fcn.data.push_back(param);
  }
  else if (!_res.fcn.function_name.compare("Bessa"))
  {
    gzmsg << "ThrusterROSPlugin::GetThrusterConversionFcn::Bessa" << std::endl;
    _res.fcn.tags.push_back("rotor_constant_l");
    this->conversionFunction->GetParam("rotor_constant_l", param);
    _res.fcn.data.push_back(param);

    _res.fcn.tags.push_back("rotor_constant_r");
    this->conversionFunction->GetParam("rotor_constant_r", param);
    _res.fcn.data.push_back(param);

    _res.fcn.tags.push_back("delta_l");
    this->conversionFunction->GetParam("delta_l", param);
    _res.fcn.data.push_back(param);

    _res.fcn.tags.push_back("delta_r");
    this->conversionFunction->GetParam("delta_r", param);
    _res.fcn.data.push_back(param);
  }
  else if (!_res.fcn.function_name.compare("LinearInterp"))
  {
    gzmsg << "ThrusterROSPlugin::GetThrusterConversionFcn::LinearInterp" << std::endl;
    std::map<double, double> table = this->conversionFunction->GetTable();

    for (auto& item : table)
    {
      gzmsg << item.first << " " << item.second << std::endl;
      _res.fcn.lookup_table_input.push_back(item.first);
      _res.fcn.lookup_table_output.push_back(item.second);
    }
  }

  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(ThrusterROSPlugin)
}
