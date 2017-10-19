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

#ifndef __CHEMICAL_CONCENTRATION_ROS_PLUGIN_HH__
#define __CHEMICAL_CONCENTRATION_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <uuv_sensor_plugins/CPCPlugin.hh>
#include <uuv_sensor_plugins_ros/SwitchableROSPlugin.hh>
#include <uuv_sensor_plugins_ros_msgs/ChemicalParticleConcentration.h>
#include <sensor_msgs/PointCloud.h>
#include <uuv_sensor_plugins/SensorPlugin.hh>
#include <string>
#include <vector>
#include <math.h>


namespace gazebo
{

class GazeboCPCROSPlugin :
  public SwitchableROSPlugin, public GazeboCPCPlugin
{
  /// \brief Constructor
  public: GazeboCPCROSPlugin();

  /// \brief Destructor
  public: virtual ~GazeboCPCROSPlugin();

  /// \brief Load module and read parameters from SDF
  protected: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Update callback from simulator.
  protected: virtual void OnPlumeParticlesUpdate(
    const sensor_msgs::PointCloud::ConstPtr &_msg);

  /// \brief Sensor update callback function
  protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

  /// \brief Input topic for the plume particle point cloud
  protected: ros::Subscriber particlesSub;

  /// \brief Output topic for the particle concentration
  protected: ros::Publisher measurementPub;

  /// \brief Flag to ensure the cloud and measurement update don't coincide
  protected: bool updatingCloud;

  /// \brief Last update from the point cloud callback
  protected: ros::Time lastUpdateTimestamp;

  /// \brief Gamma velocity parameter for the smoothing function
  protected: double gamma;

  /// \brief Noise amplitude
  protected: double noiseAmplitude;

  // \brief Radius of the kernel to identify particles that will be taken into
  // account in the concentration computation
  protected: double smoothingLength;

  /// \brief Output measurement topic
  protected: uuv_sensor_plugins_ros_msgs::ChemicalParticleConcentration outputMsg;
};

}

#endif // __CHEMICAL_CONCENTRATION_ROS_PLUGIN_HH__
