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

#include <uuv_sensor_ros_plugins/SubseaPressureROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
SubseaPressureROSPlugin::SubseaPressureROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
SubseaPressureROSPlugin::~SubseaPressureROSPlugin()
{ }

/////////////////////////////////////////////////
void SubseaPressureROSPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  GetSDFParam<double>(_sdf, "saturation", this->saturation, 3000);
  GetSDFParam<bool>(_sdf, "estimate_depth_on", this->estimateDepth, false);
  GetSDFParam<double>(_sdf, "standard_pressure", this->standardPressure,
    101.325);
  GetSDFParam<double>(_sdf, "kPa_per_meter", this->kPaPerM, 9.80638);

  this->rosSensorOutputPub =
    this->rosNode->advertise<sensor_msgs::FluidPressure>(
      this->sensorOutputTopic, 1);

  if (this->gazeboMsgEnabled)
  {
    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<sensor_msgs::msgs::Pressure>(
          this->robotNamespace + "/" + this->sensorOutputTopic, 1);
  }
}

/////////////////////////////////////////////////
bool SubseaPressureROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  // Using the world pose wrt Gazebo's ENU reference frame
  ignition::math::Vector3d pos;
#if GAZEBO_MAJOR_VERSION >= 8
  pos = this->link->WorldPose().Pos();
#else
  pos = this->link->GetWorldPose().Ign().Pos();
#endif

  double depth = std::abs(pos.Z());
  double pressure = this->standardPressure;
  if (depth >= 0)
  {
    // Convert depth to pressure
    pressure += depth * this->kPaPerM;
  }

  pressure += this->GetGaussianNoise(this->noiseAmp);

  double inferredDepth = 0.0;
  // Estimate depth, if enabled
  if (this->estimateDepth)
  {
    inferredDepth = (pressure - this->standardPressure) / this->kPaPerM;
  }

  // Publish Gazebo pressure message, if enabled
  if (this->gazeboMsgEnabled)
  {
    sensor_msgs::msgs::Pressure gazeboMsg;

    gazeboMsg.set_pressure(pressure);
    gazeboMsg.set_stddev(this->noiseSigma);

    if (this->estimateDepth)
      gazeboMsg.set_depth(inferredDepth);
    this->gazeboSensorOutputPub->Publish(gazeboMsg);
  }

  // Publish ROS pressure message
  sensor_msgs::FluidPressure rosMsg;

  rosMsg.header.stamp.sec  = _info.simTime.sec;
  rosMsg.header.stamp.nsec = _info.simTime.nsec;
  rosMsg.header.frame_id = this->link->GetName();

  rosMsg.fluid_pressure = pressure;
  rosMsg.variance = this->noiseSigma * this->noiseSigma;

  this->rosSensorOutputPub.publish(rosMsg);

  // Read the current simulation time
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastMeasurementTime = this->world->SimTime();
#else
  this->lastMeasurementTime = this->world->GetSimTime();
#endif
  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(SubseaPressureROSPlugin)
}
