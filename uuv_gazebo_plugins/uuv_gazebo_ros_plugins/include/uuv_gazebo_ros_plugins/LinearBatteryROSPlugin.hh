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

#ifndef __LINEAR_BATTERY_ROS_PLUGIN_HH__
#define __LINEAR_BATTERY_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/plugins/LinearBatteryPlugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Plugin.hh>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <string>

namespace gazebo
{

class LinearBatteryROSPlugin : public LinearBatteryPlugin
{
  /// \brief Constructor
  public: LinearBatteryROSPlugin();

  /// \brief Destructor
  public: virtual ~LinearBatteryROSPlugin();

  /// \brief Load module and read parameters from SDF.
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Initialize Module.
  public: virtual void Init();

  /// \brief Reset Module.
  public: virtual void Reset();

  /// \brief Publish battery states
  protected: void PublishBatteryState();

  /// \brief Pointer to this ROS node's handle.
  protected: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief Namespace for this ROS node
  protected: std::string robotNamespace;

  /// \brief Publisher for the dynamic state efficiency
  private: ros::Publisher batteryStatePub;

  /// \brief Battery state ROS message
  protected: sensor_msgs::BatteryState batteryStateMsg;

  /// \brief Connection for callbacks on update world.
  protected: ros::Timer updateTimer;
};

}

#endif // __LINEAR_BATTERY_ROS_PLUGIN_HH__
