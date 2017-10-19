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

#ifndef UUV_SENSOR_PLUGINS_GAZEBO_DVL_PLUGIN_H_
#define UUV_SENSOR_PLUGINS_GAZEBO_DVL_PLUGIN_H_

#include <uuv_sensor_plugins/SensorPlugin.hh>
#include "SensorDvl.pb.h"

namespace gazebo {

class GazeboDVLPlugin : public GazeboSensorPlugin
{
  /// \brief Constructor.
  public: GazeboDVLPlugin();

  /// \brief Destructor.
  public: ~GazeboDVLPlugin();

  /// \brief Load plugin and its configuration from sdf.
  protected: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Compute simulated measurements.
  protected: void SimulateMeasurement(const common::UpdateInfo& _info);

  /// \brief Update callback from simulation.
  protected: virtual bool OnUpdate(const common::UpdateInfo&);

  /// \brief Standard deviation of measurement noise (in all dimensions).
  protected: double velocityNoise;

  /// \brief Latest measured linear velocity
  protected: gazebo::math::Vector3 vel;

  /// \brief Reuse message structure sine parts (covariance) remain const.
  protected: sensor_msgs::msgs::Dvl message;
};
}

#endif  // UUV_SENSOR_PLUGINS_GAZEBO_DVL_PLUGIN_H_
