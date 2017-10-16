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

#ifndef __LINEAR_BATTERY_CONSUMER_ROS_PLUGIN_HH__
#define __LINEAR_BATTERY_CONSUMER_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <boost/scoped_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace gazebo
{

class CustomBatteryConsumerROSPlugin : public ModelPlugin
{
  /// \brief Constructor
  public: CustomBatteryConsumerROSPlugin();

  /// \brief Destructor
  public: virtual ~CustomBatteryConsumerROSPlugin();

  /// \brief Load module and read parameters from SDF.
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Callback for the device state topic subscriber
  protected: void UpdateDeviceState(const std_msgs::Bool::ConstPtr &_msg);

  /// \brief Update power load
  protected: void UpdatePowerLoad(double _powerLoad = 0.0);

  /// \brief Pointer to this ROS node's handle.
  protected: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief Subscriber to the device state flag
  protected: ros::Subscriber deviceStateSub;

  /// \brief Pointer to battery.
  protected: common::BatteryPtr battery;

  /// \brief Flag to signal whether a specific device is running
  protected: bool isDeviceOn;

  /// \brief Power load in W
  protected: double powerLoad;

  /// \brief Battery consumer ID
  protected: int consumerID;

  /// \brief Link name
  protected: std::string linkName;

  /// \brief Battery model name
  protected: std::string batteryName;

  /// \brief Connection for callbacks on update world.
  protected: event::ConnectionPtr rosPublishConnection;
};

}

#endif // __LINEAR_BATTERY_CONSUMER_ROS_PLUGIN_HH__
