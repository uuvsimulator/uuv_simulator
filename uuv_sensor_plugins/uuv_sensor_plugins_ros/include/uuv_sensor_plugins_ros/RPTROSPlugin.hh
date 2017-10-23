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

#ifndef UUV_SENSOR_PLUGINS_ROS_DVL_H_
#define UUV_SENSOR_PLUGINS_ROS_DVL_H_

#include <string>
#include <boost/scoped_ptr.hpp>
#include <uuv_sensor_plugins/RPTPlugin.hh>
#include <uuv_sensor_plugins_ros_msgs/PositionWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <uuv_sensor_plugins_ros/SwitchableROSPlugin.hh>

#include <ros/ros.h>

namespace gazebo {

/// \brief GazeboRptRosPlugin is a ROS wrapper for GazeboRPTPlugin.
/// All it does is in addition to GazeboRPTPlugin is
/// publishing simulated measurements via a ROS topic.
class GazeboRptRosPlugin :
  public GazeboRPTPlugin, public SwitchableROSPlugin {
  /// \brief Constructor.
public: GazeboRptRosPlugin();

  /// \brief Destructor.
  public: virtual ~GazeboRptRosPlugin();

  /// \brief Load module and read parameters from SDF.
  public: virtual void Load(gazebo::physics::ModelPtr _model,
                          sdf::ElementPtr _sdf);

  /// \brief Update callback from simulator.
  public: virtual bool OnUpdate(const common::UpdateInfo& _info);

  /// \brief ROS publisher for position data.
  protected: ros::Publisher pubPos;

  /// \brief ROS publisher for pose data.
  protected: ros::Publisher pubPose;

  /// \brief Reference tf frame for all position measurements.
  protected: std::string referenceFame;

  /// \brief Store message since many attributes do not change (cov.).
  protected: uuv_sensor_plugins_ros_msgs::PositionWithCovarianceStamped posMsg;

  /// \brief Store message since many attributes do not change (cov.).
  protected: geometry_msgs::PoseWithCovarianceStamped poseMsg;
};
}

#endif  // UUV_SENSOR_PLUGINS_ROS_DVL_H_
