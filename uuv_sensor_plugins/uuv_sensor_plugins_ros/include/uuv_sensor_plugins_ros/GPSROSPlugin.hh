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

#ifndef UUV_SENSOR_PLUGINS_ROS_GPS_H_
#define UUV_SENSOR_PLUGINS_ROS_GPS_H_

#include <boost/scoped_ptr.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo.hh>
#include <string>
#include <ros/ros.h>

namespace gazebo {

/// \brief GazeboGPSROSPlugin is a ROS wrapper for GpsPlugin.
/// All it does is in addition to GpsPlugin is
/// publishing simulated measurements via a ROS topic.
class GazeboGPSROSPlugin : public SensorPlugin {
  /// \brief Constructor.
  public: GazeboGPSROSPlugin();

  /// \brief Destructor.
  public: virtual ~GazeboGPSROSPlugin();

  /// \brief Load module and read parameters from SDF.
  public: virtual void Load(sensors::SensorPtr _parent,
    sdf::ElementPtr _sdf);

  /// \brief Update callback from simulator.
  public: bool OnUpdate();

  /// \brief Robot namespace
  protected: std::string robotNamespace;

  /// \brief Pointer to the parent sensor
  protected: sensors::GpsSensorPtr gazeboGPSSensor;

  /// \brief ROS node handle for communication with ROS
  protected: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief ROS publisher for GPS data.
  protected: ros::Publisher pubGPS;

  /// \brief Store dvl message since many attributes do not change (cov.).
  protected: sensor_msgs::NavSatFix gpsMessage;

  /// \brief Pointer to the update event connection.
  protected: event::ConnectionPtr updateConnection;
};
}

#endif  // UUV_SENSOR_PLUGINS_ROS_GPS_H_
