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
//
// This source code is derived from hector_gazebo
//   (https://github.com/tu-darmstadt-ros-pkg/hector_gazebo)
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt,
// licensed under the BSD 3-Clause license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// The original code was modified to:
// - be more consistent with other sensor plugins within uuv_simulator,
// - remove its dependency on ROS. The ROS interface is instead implemented in
//   a derived class within the uuv_sensor_plugins_ros package,
// - adhere to Gazebo's coding standards.


#ifndef UUV_SENSOR_PLUGINS_GAZEBO_MAGNETOMETER_PLUGIN_H_
#define UUV_SENSOR_PLUGINS_GAZEBO_MAGNETOMETER_PLUGIN_H_

#include <Eigen/Core>

#include "SensorMagnetic.pb.h"

#include "SensorPlugin.hh"

namespace gazebo {

struct MagnetometerParameters {
    /// \brief Intensity of reference earth magnetic field [muT].
    double intensity;
    /// \brief Heading angle of reference earth magnetic field [rad].
    double heading;
    /// \brief Declination of reference earth magnetic field [rad].
    double declination;
    /// \brief Inclination of reference earth magnetic field [rad].
    double inclination;

    /// \brief Discrete-time standard dev. of output noise in xy-axis [muT].
    double noiseXY;
    /// \brief Discrete-time standard dev. of output noise in z-axis [muT].
    double noiseZ;

    /// \brief Stddev of constant systematic offset of measurements [muT].
    double turnOnBias;
};

/// \brief GazeboMagnetometerPlugin simulates a magnetometer sensor
class GazeboMagnetometerPlugin : public GazeboSensorPlugin
{
  /// \brief Constructor.
  public: GazeboMagnetometerPlugin();

  /// \brief Destructor.
  public: ~GazeboMagnetometerPlugin();

  /// \brief Load plugin and its configuration from sdf.
  protected: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Compute simulated measurements.
  protected: void SimulateMeasurement(const common::UpdateInfo& info);

  /// \brief Update callback from simulation.
  protected: virtual bool OnUpdate(const common::UpdateInfo&);

  /// \brief Magnetometer configuration parameters:
  protected: MagnetometerParameters parameters_;

  /// \brief Reference magnetic field in world frame:
  protected: gazebo::math::Vector3 magneticFieldWorld_;

  /// \brief Constant turn-on bias [muT].
  protected: gazebo::math::Vector3 turnOnBias_;

  /// \brief Last measurement of magnetic field
  protected: gazebo::math::Vector3 measMagneticField_;

  /// \brief Gazebo sensor message. Kept as member since largely constant.
  protected: sensor_msgs::msgs::Magnetic magneticGazeboMessage_;
};
}

#endif  // UUV_SENSOR_PLUGINS_GAZEBO_MAGNETOMETER_PLUGIN_H_
