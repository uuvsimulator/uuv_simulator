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

#ifndef SWITCHABLE_ROS_GPU_RAY_H_
#define SWITCHABLE_ROS_GPU_RAY_H_

#include <boost/scoped_ptr.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <uuv_sensor_plugins_ros/SwitchableROSSensorPlugin.hh>

namespace gazebo {

/// \brief GazeboGpsRosPlugin is a ROS wrapper for GpsPlugin.
/// All it does is in addition to GpsPlugin is
/// publishing simulated measurements via a ROS topic.
class SwitchableROSGPURaySensor : public SwitchableROSSensorPlugin {
  /// \brief Constructor
  public: SwitchableROSGPURaySensor();

  /// \brief Destructor
  public: virtual ~SwitchableROSGPURaySensor();

  /// \brief Load module and read parameters from SDF.
  public: virtual void Load(sensors::SensorPtr _parent,
    sdf::ElementPtr _sdf);

  /// \brief Input laser scan message
  protected: sensor_msgs::LaserScan msg;

  public: void UpdateSensorInput(const sensor_msgs::LaserScan::ConstPtr &_msg);

};

}

#endif // SWITCHABLE_ROS_GPU_RAY_H_
