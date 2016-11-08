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

#ifndef UUV_SENSOR_PLUGINS_GAZEBO_SUBSEAPRESSURE_PLUGIN_H_
#define UUV_SENSOR_PLUGINS_GAZEBO_SUBSEAPRESSURE_PLUGIN_H_

#include "SensorPlugin.hh"

namespace gazebo {

class GazeboSubseaPressurePlugin : public GazeboSensorPlugin
{
  /// \brief Constructor.
  public: GazeboSubseaPressurePlugin();

  /// \brief Destructor.
  public: ~GazeboSubseaPressurePlugin();

  /// \brief Load plugin and its configuration from sdf.
  protected: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Compute simulated measurements.
  protected: void SimulateMeasurement(const common::UpdateInfo& _info);

  /// \brief Update callback from simulation.
  protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

  /// \brief Measurement Range: Maximum Pressure, min is 0 [kPa].
  protected: double range_;

  /// \brief: Measurement standard deviation [kPa].
  protected: double stdDev_;

  /// \brief Measured Pressure [kPa].
  protected: double measuredPressure_;

  /// \brief Optional: Inferred depth [m].
  protected: double inferredDepth_;

  /// \brief Does this sensor infer and output a depth measurement?
  protected: bool depthEstimation_;

  /// \brief Sea level atmospheric pressure (at z == 0) [kPa].
  protected: double standardPressure_;

  /// \brief Increase in pressure [kPa] per depth [m]:
  protected: double kPaPerM_;
};
}

#endif  // UUV_SENSOR_PLUGINS_GAZEBO_SUBSEAPRESSURE_PLUGIN_H_
