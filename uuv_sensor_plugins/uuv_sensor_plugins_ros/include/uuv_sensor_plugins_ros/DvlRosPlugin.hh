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

#include <boost/scoped_ptr.hpp>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <uuv_sensor_plugins/DvlPlugin.hh>
#include <uuv_sensor_plugins_ros_msgs/DVL.h>

#include <ros/ros.h>

namespace gazebo {

/// \brief GazeboDvlRosPlugin is a ROS wrapper for GazeboDvlPlugin.
/// All it does is in addition to GazeboDvlPlugin is
/// publishing simulated measurements via a ROS topic.
class GazeboDVLRosPlugin : public GazeboDvlPlugin {
  /// \brief Constructor.
public: GazeboDVLRosPlugin();

  /// \brief Destructor.
  public: virtual ~GazeboDVLRosPlugin();

  /// \brief Load module and read parameters from SDF.
  public: virtual void Load(gazebo::physics::ModelPtr _model,
                          sdf::ElementPtr _sdf);

  /// \brief Update callback from simulator.
  public: virtual bool OnUpdate(const common::UpdateInfo& _info);

  /// \brief ROS node handle for communication with ROS
  protected: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief ROS publisher for dvl data.
  protected: ros::Publisher pubDvl;

  /// \brief ROS publisher for twist data.
  protected: ros::Publisher pubTwist;

  /// \brief Store dvl message since many attributes do not change (cov.).
  protected: uuv_sensor_plugins_ros_msgs::DVL dvlMessage;

  /// \brief Store pose message since many attributes do not change (cov.).
  protected: geometry_msgs::TwistWithCovarianceStamped twistMessage;
};
}

#endif  // UUV_SENSOR_PLUGINS_ROS_DVL_H_
