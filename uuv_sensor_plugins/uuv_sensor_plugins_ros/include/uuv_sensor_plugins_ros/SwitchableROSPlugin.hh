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

#ifndef SWITCHABLE_ROS_PLUGIN_H_
#define SWITCHABLE_ROS_PLUGIN_H_

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <uuv_sensor_plugins_ros_msgs/ChangeSensorState.h>

#define UUV_SWITCHABLE_SUFFIX "_switchable"

namespace gazebo {

/// \brief SwitchableROSPlugin
/// Provides a service to turn the sensor on and off
/// TODO: Find a better solution to switch payload on and off, Gazebo's
/// IsActive still not implemented properly
class SwitchableROSPlugin {
  /// \brief Constructor
  public: SwitchableROSPlugin();

  /// \brief Destructor
  public: virtual ~SwitchableROSPlugin();

  /// \brief Initialization function to create necessary services and topics
  protected: bool InitSwitchablePlugin(std::string _topicPrefix,
    bool _isOn = true);

  /// \brief Topic and services prefix string
  protected: std::string topicNamePrefix;

  /// \brief Flag to control the generation of output messages
  protected: std_msgs::Bool isOn;

  /// \brief ROS node handle for communication with ROS
  protected: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief Pointer to the update event connection.
  protected: event::ConnectionPtr updateConnection;

  /// \brief Service server object
  protected: ros::ServiceServer changeSensorSrv;

  /// \brief ROS publisher for the switchable sensor data
  protected: ros::Publisher pluginStatePub;

  /// \brief Returns true if the plugin is activated
  protected: bool IsOn();

  /// \brief Publish the current state of the plugin
  protected: void PublishState();

  /// \brief Change sensor state (ON/OFF)
  public: bool ChangeSensorState(
      uuv_sensor_plugins_ros_msgs::ChangeSensorState::Request& _req,
      uuv_sensor_plugins_ros_msgs::ChangeSensorState::Response& _res);
};

}

#endif // SWITCHABLE_ROS_PLUGIN_H_
