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

#include <uuv_sensor_ros_plugins/CPCROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
CPCROSPlugin::CPCROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
CPCROSPlugin::~CPCROSPlugin()
{ }

/////////////////////////////////////////////////
void CPCROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  std::string inputTopic;
  GetSDFParam<std::string>(_sdf, "plume_topic", inputTopic, "");
  GZ_ASSERT(!inputTopic.empty(), "Plume topic for the point cloud is empty!");

  GetSDFParam<double>(_sdf, "gamma", this->gamma, 0.001);
  GZ_ASSERT(this->gamma > 0, "Gamma value must be greater than zero");

  GetSDFParam<double>(_sdf, "radius", this->smoothingLength, 3.0);
  GZ_ASSERT(this->smoothingLength > 0,
    "Radius of the sensor must be greater than zero");

  GetSDFParam<double>(_sdf, "noise_amplitude", this->noiseAmplitude, 0.0);
    GZ_ASSERT(this->noiseAmplitude >= 0.0,
      "Signal noise amplitude must be greater or equal to zero");

  this->particlesSub = this->rosNode->subscribe<sensor_msgs::PointCloud>(
    inputTopic, 1,
    boost::bind(&CPCROSPlugin::OnPlumeParticlesUpdate,
      this, _1));

  // Set initial particle concentration value to zero
  this->outputMsg.concentration = 0.0;

  // Set the is_measuring flag initially to false
  this->outputMsg.is_measuring = false;

  this->rosSensorOutputPub = this->rosNode->advertise<
    uuv_sensor_plugins_ros_msgs::ChemicalParticleConcentration>(
      this->sensorOutputTopic, 1);

  this->lastUpdateTimestamp = ros::Time::now();

  gzmsg << "CPCROSPlugin[" << this->link->GetName()
    << "] initialized!" << std::endl
    << "\t- Input topic= " << inputTopic << std::endl
    << "\t- Output topic= " << this->sensorOutputTopic << std::endl;
}

/////////////////////////////////////////////////
bool CPCROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info) || this->updatingCloud)
    return false;

  // Set particle concentration to zero if the point cloud message has not
  // been received for a long time
  if (_info.simTime.Double() - this->lastUpdateTimestamp.toSec() > 5.0)
  {
    this->outputMsg.is_measuring = false;
    this->outputMsg.concentration = 0.0;
  }

  this->outputMsg.header.frame_id = this->referenceFrameID;
  this->outputMsg.concentration += this->GetGaussianNoise(
    this->noiseAmplitude);
  this->outputMsg.header.stamp.sec = _info.simTime.sec;
  this->outputMsg.header.stamp.nsec = _info.simTime.nsec;
  this->rosSensorOutputPub.publish(this->outputMsg);

  // Read the current simulation time
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastMeasurementTime = this->world->SimTime();
#else
  this->lastMeasurementTime = this->world->GetSimTime();
#endif
  return true;
}

/////////////////////////////////////////////////
void CPCROSPlugin::OnPlumeParticlesUpdate(
  const sensor_msgs::PointCloud::ConstPtr &_msg)
{
  if (this->rosSensorOutputPub.getNumSubscribers() > 0)
  {
    this->updatingCloud = true;

    double totalParticleConc = 0.0;
    double smoothingParam;
    double particleConc;
    double distToParticle;

    ignition::math::Vector3d linkPos, linkPosRef;
#if GAZEBO_MAJOR_VERSION >= 8
    linkPos = this->link->WorldPose().Pos();
#else
    linkPos = this->link->GetWorldPose().Ign().Pos();
#endif

    // Apply transformation wrt to the reference frame
    linkPosRef = linkPos - this->referenceFrame.Pos();
    linkPosRef = this->referenceFrame.Rot().RotateVectorReverse(linkPosRef);

    this->outputMsg.is_measuring = true;
    // Store the current position wrt the reference frame where this
    // measurement was taken
    this->outputMsg.position.x = linkPosRef.X();
    this->outputMsg.position.y = linkPosRef.Y();
    this->outputMsg.position.z = linkPosRef.Z();

    // Calculate the current position in WGS84 spherical coordinates
    ignition::math::Vector3d cartVec = linkPos;
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d scVec =
      this->link->GetWorld()->SphericalCoords()->SphericalFromLocal(
        cartVec);
#else
    ignition::math::Vector3d scVec =
      this->link->GetWorld()->GetSphericalCoordinates()->SphericalFromLocal(
        cartVec);
#endif
    this->outputMsg.latitude = scVec.X();
    this->outputMsg.longitude = scVec.Y();
    this->outputMsg.depth = -1 * scVec.Z();

    // Store this measurement's time stamp
    this->lastUpdateTimestamp = _msg->header.stamp;

    double currentTime = _msg->header.stamp.toSec();

    double initSmoothingLength = std::pow(this->smoothingLength, 2.0 / 3);
    ignition::math::Vector3d particle;
    for (int i = 0; i < _msg->points.size(); i++)
    {
      particle = ignition::math::Vector3d(_msg->points[i].x, _msg->points[i].y,
        _msg->points[i].z);

      smoothingParam = std::pow(initSmoothingLength +
        this->gamma * (currentTime - _msg->channels[0].values[i]), 1.5);
      distToParticle = linkPos.Distance(particle);

      // Compute particle concentration
      if (distToParticle >= 0 && distToParticle < smoothingParam)
        particleConc = 4.0 -
          6.0 * std::pow(distToParticle / smoothingParam, 2) +
          3.0 * std::pow(distToParticle / smoothingParam, 3);
      else if (distToParticle >= smoothingParam && distToParticle < 2 * smoothingParam)
        particleConc = std::pow(2 - distToParticle / smoothingParam, 3);
      else
        particleConc = 0.0;

      particleConc *= 1 / (4 * M_PI * std::pow(smoothingParam, 3));
      totalParticleConc += particleConc;
    }

    this->outputMsg.concentration = totalParticleConc;
    this->updatingCloud = false;
  }
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(CPCROSPlugin)
}
