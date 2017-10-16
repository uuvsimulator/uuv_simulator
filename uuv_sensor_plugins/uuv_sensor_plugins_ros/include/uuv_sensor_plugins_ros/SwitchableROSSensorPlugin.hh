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

#ifndef SWITCHABLE_ROS_SENSOR_PLUGIN_H_
#define SWITCHABLE_ROS_SENSOR_PLUGIN_H_

#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo.hh>
#include <string>
#include <ros/ros.h>
#include <uuv_sensor_plugins_ros/SwitchableROSPlugin.hh>

#define UUV_SWITCHABLE_SUFFIX "_switchable"

namespace gazebo {

/// \brief SwitchableROSSensorPlugin
/// Provides a service to turn the sensor on and off
/// TODO: Find a better solution to switch payload on and off, Gazebo's IsActive still not implemented properly
class SwitchableROSSensorPlugin : public SensorPlugin, public SwitchableROSPlugin {
  /// \brief Constructor
  public: SwitchableROSSensorPlugin();

  /// \brief Destructor
  public: virtual ~SwitchableROSSensorPlugin();

  /// \brief Load module and read parameters from SDF.
  public: virtual void Load(sensors::SensorPtr _parent,
    sdf::ElementPtr _sdf);

  /// \brief Namespace of the vehicle
  protected: std::string robotNamespace;

  /// \brief Name of the input topic message
  protected: std::string inputTopicName;

  /// \brief Pointer to the parent sensor
  protected: sensors::SensorPtr parentSensor;

  /// \brief ROS subscriber from the parent sensor
  protected: ros::Subscriber inputSub;

  /// \brief ROS publisher for the switchable sensor data
  protected: ros::Publisher outputPub;

};

}

#endif // SWITCHABLE_ROS_SENSOR_PLUGIN_H_
