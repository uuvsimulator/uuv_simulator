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

#ifndef __ROS_BASE_SENSOR_PLUGIN_HH__
#define __ROS_BASE_SENSOR_PLUGIN_HH__

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <uuv_sensor_ros_plugins/ROSBasePlugin.hh>
#include <boost/bind.hpp>
#include <string>

namespace gazebo
{
  class ROSBaseSensorPlugin : public ROSBasePlugin, public SensorPlugin
  {
    /// \brief Class constructor
    public: ROSBaseSensorPlugin();

    /// \brief Class destructor
    public: virtual ~ROSBaseSensorPlugin();

    /// \brief Load plugin and its configuration from sdf,
    protected: virtual void Load(sensors::SensorPtr _model,
      sdf::ElementPtr _sdf);

    /// \brief Update callback from simulation.
    protected: virtual bool OnUpdate(const common::UpdateInfo&);

    /// \brief Pointer to the parent sensor
    protected: sensors::SensorPtr parentSensor;
  };
}

#endif // __ROS_BASE_SENSOR_PLUGIN_HH__
