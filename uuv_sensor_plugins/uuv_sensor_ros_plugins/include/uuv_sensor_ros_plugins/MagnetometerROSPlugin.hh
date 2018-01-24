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
// - adhere to Gazebo's coding standards.

#ifndef __UUV_MAGNETOMETER_ROS_PLUGIN_HH__
#define __UUV_MAGNETOMETER_ROS_PLUGIN_HH__

#include "SensorMagnetic.pb.h"
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <sensor_msgs/MagneticField.h>

namespace gazebo
{
  struct MagnetometerParameters
  {
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
    /// \brief Standard deviation of constant systematic offset of
    /// measurements [muT].
    double turnOnBias;
  };

  class MagnetometerROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: MagnetometerROSPlugin();

    /// \brief Class destructor
    public: virtual ~MagnetometerROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Magnetometer configuration parameters:
    protected: MagnetometerParameters parameters;

    /// \brief Reference magnetic field in world frame:
    protected: ignition::math::Vector3d magneticFieldWorld;

    /// \brief Constant turn-on bias [muT].
    protected: ignition::math::Vector3d turnOnBias;

    /// \brief Last measurement of magnetic field
    protected: ignition::math::Vector3d measMagneticField;

    /// \brief ROS message
    protected: sensor_msgs::MagneticField rosMsg;
  };
}

#endif // __UUV_MAGNETOMETER_ROS_PLUGIN_HH__
