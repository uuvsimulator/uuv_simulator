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

#include <uuv_sensor_plugins/SubseaPressurePlugin.hh>

#include <gazebo/physics/physics.hh>

#include "SensorPressure.pb.h"

#include "Common.hh"

namespace gazebo {

GazeboSubseaPressurePlugin::GazeboSubseaPressurePlugin()
    : GazeboSensorPlugin()
{
}

GazeboSubseaPressurePlugin::~GazeboSubseaPressurePlugin()
{
}

void GazeboSubseaPressurePlugin::Load(physics::ModelPtr _model,
                                      sdf::ElementPtr _sdf)
{
    GazeboSensorPlugin::Load(_model, _sdf);

    getSdfParam<double>(_sdf, "range", this->range_, 3000);
    getSdfParam<double>(_sdf, "stddev", this->stdDev_, 0.1);
    getSdfParam<bool>(_sdf, "estimateDepth", this->depthEstimation_, false);
    getSdfParam<double>(_sdf, "standardPressure", this->standardPressure_,
                        101.325);
    getSdfParam<double>(_sdf, "kPaPerM", this->kPaPerM_, 9.80638);

    if (this->sensorTopic_.empty())
        this->sensorTopic_ = "pressure";
    this->publisher_ = nodeHandle_->Advertise<sensor_msgs::msgs::Pressure>(
        this->sensorTopic_, 10);
}

void GazeboSubseaPressurePlugin::SimulateMeasurement(
        const common::UpdateInfo &_info)
{
    // depth is positive for nagative z
    double depth = -this->link_->GetWorldPose().pos.z;

    // convert depth to pressure
    double pressure = this->standardPressure_;
    if (depth > 0.)
    {
        pressure += depth * this->kPaPerM_;
    }
    pressure += this->stdDev_*this->normal_(this->rndGen_);

    this->measuredPressure_ = pressure;
    if (this->depthEstimation_)
    {
        this->inferredDepth_ = (pressure - this->standardPressure_)
                / this->kPaPerM_;
    }

    this->lastMeasTime_ = _info.simTime;
}

bool GazeboSubseaPressurePlugin::OnUpdate(const common::UpdateInfo &_info)
{
    if (!ShouldIGenerate(_info))
        return false;

    SimulateMeasurement(_info);

    sensor_msgs::msgs::Pressure message;

    message.set_pressure(this->measuredPressure_);
    message.set_stddev(this->stdDev_);

    if (this->depthEstimation_)
    {
        message.set_depth(this->inferredDepth_);
    }

    publisher_->Publish(message);

    return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboSubseaPressurePlugin);
}
