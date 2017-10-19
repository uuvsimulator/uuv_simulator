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

// This source code is derived from rotors_simulator
//   (https://github.com/ethz-asl/rotors_simulator)
// * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland,
// * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland,
// * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland,
// * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland,
// * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland,
// licensed under the Apache-2.0 license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// The original code was modified to:
// - be more consistent with other sensor plugins within uuv_simulator,
// - remove its dependency on ROS. The ROS interface is instead implemented in
//   a derived class within the uuv_sensor_plugins_ros package,
// - adhere to Gazebo's coding standards.


#ifndef UUV_SENSOR_PLUGINS_GAZEBO_IMU_PLUGIN_H_
#define UUV_SENSOR_PLUGINS_GAZEBO_IMU_PLUGIN_H_

#include <random>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/msgs/msgs.hh"
#include "SensorImu.pb.h"

#include "SensorPlugin.hh"

namespace gazebo {

// Default values for use with ADIS16448 IMU
static constexpr double kDefaultAdisGyroscopeNoiseDensity =
    2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk =
    2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime =
    1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma =
    0.5 / 180.0 * M_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity =
    2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerRandomWalk =
    2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime =
    300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma =
    20.0e-3 * 9.8;
static constexpr double kDefaultOrientationNoise = 0.5;

static const std::string kDefaultImuTopic = "imu";

/// \brief ImuParameters stores all IMU model parameters.
/// A description of these parameters can be found here:
/// https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
struct ImuParameters {
  /// \brief Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
  double gyroscopeNoiseDensity;
  /// \brief Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
  double gyroscopeRandomWalk;
  /// \brief Gyroscope bias correlation time constant [s]
  double gyroscopeBiasCorrelationTime;
  /// \brief Gyroscope turn on bias standard deviation [rad/s]
  double gyroscopeTurnOnBiasSigma;
  /// \brief Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
  double accelerometerNoiseDensity;
  /// \brief Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
  double accelerometerRandomWalk;
  /// \brief Accelerometer bias correlation time constant [s]
  double accelerometerBiasCorrelationTime;
  /// \brief Accelerometer turn on bias standard deviation [m/s^2]
  double accelerometerTurnOnBiasSigma;

  /// \brief Standard deviation of orientation noise per axis [rad].
  double orientationNoise;

  /// \brief Constructor.
  ImuParameters()
      : gyroscopeNoiseDensity(kDefaultAdisGyroscopeNoiseDensity),
        gyroscopeRandomWalk(kDefaultAdisGyroscopeRandomWalk),
        gyroscopeBiasCorrelationTime(
            kDefaultAdisGyroscopeBiasCorrelationTime),
        gyroscopeTurnOnBiasSigma(kDefaultAdisGyroscopeTurnOnBiasSigma),
        accelerometerNoiseDensity(kDefaultAdisAccelerometerNoiseDensity),
        accelerometerRandomWalk(kDefaultAdisAccelerometerRandomWalk),
        accelerometerBiasCorrelationTime(
            kDefaultAdisAccelerometerBiasCorrelationTime),
        accelerometerTurnOnBiasSigma(
            kDefaultAdisAccelerometerTurnOnBiasSigma),
        orientationNoise(kDefaultOrientationNoise)
  {}
};

/// \brief GazeboIMUPlugin simulates an IMU with drifting gyro. and accel.
///        The implementation is based on the GazeboIMUPlugin found in the
///        rotors simulator.
class GazeboIMUPlugin : public GazeboSensorPlugin {
  /// \brief Constructor.
  public: GazeboIMUPlugin();

  /// \brief Desctructor.
  public: virtual ~GazeboIMUPlugin();

  /// \brief Load plugin and its configuration from sdf,
  protected: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Apply and add nosie model to ideal measurements.
  protected: void AddNoise(Eigen::Vector3d &linear_acceleration,
                           Eigen::Vector3d &angular_velocity,
                           Eigen::Quaterniond &orientation,
                           double dt);

  /// \brief Generate a simulated measurement.
  protected: void SimulateMeasurement(const common::UpdateInfo& _info);

  /// \brief Update callback from simulation.
  protected: virtual bool OnUpdate(const common::UpdateInfo&);

  /// \brief Last measurement of linear acceleration..
  protected: Eigen::Vector3d measLinearAcc_;

  /// \brief Last measurement of angular velocity.
  protected: Eigen::Vector3d measAngularVel_;

  /// \brief (Simulation) time when the last sensor measurement was generated.
  protected: Eigen::Quaterniond measOrientation_;

  /// \brief Gazebo sensor message. Kept as member since largely constant.
  protected: sensor_msgs::msgs::Imu imuGazeboMessage_;

  /// \brief Gravity vector wrt. world frame according to physics engine.
  protected: math::Vector3 gravityW_;

  /// \brief Current (drifting) gyroscope bias.
  protected: Eigen::Vector3d gyroscopeBias_;

  /// \brief Current (drifting) accelerometer bias.
  protected: Eigen::Vector3d accelerometerBias_;

  /// \brief Constant turn-on gyroscope bias.
  protected: Eigen::Vector3d gyroscopeTurnOnBias_;

  /// \brief Constant turn-on accelerometer bias.
  protected: Eigen::Vector3d accelerometerTurnOnBias_;

  /// \brief IMU model parameters.
  protected: ImuParameters imuParameters_;
};
}

#endif  // UUV_SENSOR_PLUGINS_GAZEBO_IMU_PLUGIN_H_
