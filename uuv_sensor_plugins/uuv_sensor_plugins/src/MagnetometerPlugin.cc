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

#include <uuv_sensor_plugins/MagnetometerPlugin.hh>

#include <gazebo/physics/physics.hh>

#include "Common.hh"

namespace gazebo {

GazeboMagnetometerPlugin::GazeboMagnetometerPlugin()
    : GazeboSensorPlugin()
{
}

GazeboMagnetometerPlugin::~GazeboMagnetometerPlugin()
{
}

void GazeboMagnetometerPlugin::Load(physics::ModelPtr _model,
                                    sdf::ElementPtr _sdf)
{
    GazeboSensorPlugin::Load(_model, _sdf);

    // Load settings specific to this type of sensor
    getSdfParam<double>(_sdf, "intensity", parameters_.intensity, 1.0);
    getSdfParam<double>(_sdf, "referenceHeading", parameters_.heading, M_PI);
    getSdfParam<double>(_sdf, "declination", parameters_.declination, 0.0);
    getSdfParam<double>(_sdf, "inclination", parameters_.inclination,
                        60.*M_PI/180.);
    getSdfParam<double>(_sdf, "noise_xy", parameters_.noiseXY, 1.0);
    getSdfParam<double>(_sdf, "noise_z", parameters_.noiseZ, 1.4);
    getSdfParam<double>(_sdf, "turn_on_bias", parameters_.turnOnBias, 2.0);

    // Now Prepare derived values

    // Note: Gazebo uses NorthWestUp coordinate system,
    // heading and declination are compass headings
    magneticFieldWorld_.x = parameters_.intensity*cos(parameters_.inclination)
            * cos(parameters_.heading - parameters_.declination);
    magneticFieldWorld_.y = parameters_.intensity*cos(parameters_.inclination)
            * sin(parameters_.heading - parameters_.declination);
    magneticFieldWorld_.z = parameters_.intensity
            *(-sin(parameters_.inclination));

    turnOnBias_.x = parameters_.turnOnBias *
            normal_(rndGen_);
    turnOnBias_.y = parameters_.turnOnBias *
            normal_(rndGen_);
    turnOnBias_.z = parameters_.turnOnBias *
            normal_(rndGen_);


    // Fill constant fields of sensor message already
    // TODO: This should better be a 3x3 matrix instead
    magneticGazeboMessage_.add_magnetic_field_covariance(
                parameters_.noiseXY*parameters_.noiseXY);
    magneticGazeboMessage_.add_magnetic_field_covariance(
                parameters_.noiseXY*parameters_.noiseXY);
    magneticGazeboMessage_.add_magnetic_field_covariance(
                parameters_.noiseZ*parameters_.noiseZ);

    if (this->sensorTopic_.empty())
        this->sensorTopic_ = "magnetometer";

    publisher_ = nodeHandle_->Advertise<sensor_msgs::msgs::Magnetic>(
                sensorTopic_, 10);
}

void GazeboMagnetometerPlugin::SimulateMeasurement(
        const common::UpdateInfo& info)
{
    math::Pose pose = link_->GetWorldPose();

    math::Vector3 noise(parameters_.noiseXY*normal_(rndGen_),
                parameters_.noiseXY*normal_(rndGen_),
                parameters_.noiseZ*normal_(rndGen_));

    measMagneticField_ = pose.rot.RotateVectorReverse(magneticFieldWorld_)
            + turnOnBias_ + noise;
}

bool GazeboMagnetometerPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    if (!ShouldIGenerate(_info))
        return false;

    SimulateMeasurement(_info);

    gazebo::msgs::Vector3d* field = new gazebo::msgs::Vector3d();
    field->set_x(this->measMagneticField_.x);
    field->set_y(this->measMagneticField_.y);
    field->set_z(this->measMagneticField_.z);
    this->magneticGazeboMessage_.set_allocated_magnetic_field(field);

    publisher_->Publish(magneticGazeboMessage_);

    return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMagnetometerPlugin);
}
