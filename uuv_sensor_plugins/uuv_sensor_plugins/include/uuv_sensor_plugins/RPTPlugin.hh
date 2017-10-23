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

#ifndef UUV_SENSOR_PLUGINS_GAZEBO_RPT_PLUGIN_H_
#define UUV_SENSOR_PLUGINS_GAZEBO_RPT_PLUGIN_H_

#include "SensorPlugin.hh"
#include "SensorRpt.pb.h"

namespace gazebo {

class GazeboRPTPlugin : public GazeboSensorPlugin
{
  /// \brief Constructor.
  public: GazeboRPTPlugin();

  /// \brief Destructor.
  public: ~GazeboRPTPlugin();

  /// \brief Load plugin and its configuration from sdf.
  protected: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Compute simulated measurements.
  protected: void SimulateMeasurement(const common::UpdateInfo& _info);

  /// \brief Update callback from simulation.
  protected: virtual bool OnUpdate(const common::UpdateInfo&);

  /// \brief Standard deviation of measurement noise (in all dimensions).
  protected: double positionNoise;

  /// \brief Latest measured position.
  protected: gazebo::math::Vector3 position;

  /// \brief Reuse message structure sine parts (covariance) remain const.
  protected: sensor_msgs::msgs::Rpt message;
};
}

#endif  // UUV_SENSOR_PLUGINS_GAZEBO_RPT_PLUGIN_H_
