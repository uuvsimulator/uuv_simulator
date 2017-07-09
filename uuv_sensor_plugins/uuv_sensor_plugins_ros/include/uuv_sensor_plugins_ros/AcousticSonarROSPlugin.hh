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

#ifndef UUV_SENSOR_PLUGINS_ROS_ACOUSTIC_SONAR_H_
#define UUV_SENSOR_PLUGINS_ROS_ACOUSTIC_SONAR_H_

#include <boost/scoped_ptr.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo.hh>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <uuv_sensor_plugins_ros_msgs/ChangeSensorState.h>

namespace gazebo {

/// \brief GazeboGpsRosPlugin is a ROS wrapper for GpsPlugin.
/// All it does is in addition to GpsPlugin is
/// publishing simulated measurements via a ROS topic.
class GazeboAcousticSonarRosPlugin : public SensorPlugin {
  /// \brief Constructor
  public: GazeboAcousticSonarRosPlugin();

  /// \brief Destructor
  public: virtual ~GazeboAcousticSonarRosPlugin();

  /// \brief Load module and read parameters from SDF.
  public: virtual void Load(sensors::SensorPtr _parent,
    sdf::ElementPtr _sdf);

  protected: bool sensorOn;

  /// \brief Namespace of the vehicle
  protected: std::string vehicleNamespace;

  protected: std::string inputTopicName;

  protected: sensor_msgs::LaserScan msg;

  /// \brief Pointer to the parent sensor
  protected: sensors::GpuRaySensorPtr gazeboSensor;

  /// \brief ROS node handle for communication with ROS
  protected: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection;

  /// \brief Service server object
  public: ros::ServiceServer changeSensorSrv;

  /// \brief ROS publisher for GPS data.
  protected: ros::Subscriber laserScanInputSub;

  /// \brief ROS publisher for GPS data.
  protected: ros::Publisher laserScanInputPub;

  public: void UpdateSensorInput(const sensor_msgs::LaserScan::ConstPtr &_msg);

  /// \brief Change sensor state (ON/OFF)
  public: bool ChangeSensorState(
      uuv_sensor_plugins_ros_msgs::ChangeSensorState::Request& _req,
      uuv_sensor_plugins_ros_msgs::ChangeSensorState::Response& _res);
};

}

#endif // UUV_SENSOR_PLUGINS_ROS_ACOUSTIC_SONAR_H_
