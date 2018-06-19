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

#ifndef __UUV_SUBSEA_PRESSURE_ROS_PLUGIN_HH__
#define __UUV_SUBSEA_PRESSURE_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include "SensorPressure.pb.h"
#include <sensor_msgs/FluidPressure.h>

namespace gazebo
{
  class SubseaPressureROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: SubseaPressureROSPlugin();

    /// \brief Class destructor
    public: ~SubseaPressureROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Sensor saturation (max. value for output pressure in Pa)
    protected: double saturation;

    /// \brief If flag is set to true, estimate depth according to pressure
    /// measurement
    protected: bool estimateDepth;

    /// \brief Standard pressure
    protected: double standardPressure;

    /// \brief Factor of kPa per meter
    protected: double kPaPerM;
  };
}

#endif // __UUV_SUBSEA_PRESSURE_ROS_PLUGIN_HH__
