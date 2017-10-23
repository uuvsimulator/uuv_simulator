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

#include <uuv_sensor_plugins_ros/CPCROSPlugin.hh>

namespace gazebo
{

/////////////////////////////////////////////////
GazeboCPCROSPlugin::GazeboCPCROSPlugin()
  : GazeboCPCPlugin()
{ }

/////////////////////////////////////////////////
GazeboCPCROSPlugin::~GazeboCPCROSPlugin()
{ }

/////////////////////////////////////////////////
void GazeboCPCROSPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  try
  {
    GazeboCPCPlugin::Load(_model, _sdf);
  }
  catch(gazebo::common::Exception &_e)
  {
    gzerr << "Error loading GazeboCPCPlugin" << std::endl;
    return;
  }

  if (!ros::isInitialized())
  {
    gzerr << "Not loading GazeboCPCROSPlugin since ROS has "
          << "not been properly initialized." << std::endl;
    return;
  }

  this->rosNode.reset(new ros::NodeHandle(this->namespace_));

  GZ_ASSERT(_sdf->HasElement("plumeTopic"), "Plume topic name is missing");
  std::string inputTopic = _sdf->Get<std::string>("plumeTopic");
  GZ_ASSERT(!inputTopic.empty(), "Plume topic for the point cloud is empty!");

  this->gamma = 0.001;
  if (_sdf->HasElement("gamma"))
    this->gamma = _sdf->Get<double>("gamma");

  GZ_ASSERT(this->gamma > 0, "Gamma value must be greater than zero");

  this->smoothingLength = 3.0;
  if (_sdf->HasElement("radius"))
    this->smoothingLength = _sdf->Get<double>("radius");

  GZ_ASSERT(this->smoothingLength > 0,
    "Radius of the sensor must be greater than zero");

  this->noiseAmplitude = 0.0;
  if (_sdf->HasElement("noiseAmplitude"))
    this->noiseAmplitude = _sdf->Get<double>("noiseAmplitude");

  GZ_ASSERT(this->noiseAmplitude >= 0.0,
    "Signal noise amplitude must be greater or equal to zero");

  this->particlesSub = this->rosNode->subscribe<sensor_msgs::PointCloud>(
    inputTopic, 1,
    boost::bind(&GazeboCPCROSPlugin::OnPlumeParticlesUpdate,
      this, _1));

  // Set initial particle concentration value to zero
  this->outputMsg.concentration = 0.0;

  // Initialize the time stamp of the point cloud update
  this->lastUpdateTimestamp = ros::Time::now();

  // Set the is_measuring flag initially to false
  this->outputMsg.is_measuring = false;

  this->measurementPub = this->rosNode->advertise<
    uuv_sensor_plugins_ros_msgs::ChemicalParticleConcentration>(
      this->sensorTopic_, 1);

  // Initialize the switchable functionality of the sensor
  bool isSensorOn = true;
  if (_sdf->HasElement("is_on"))
    isSensorOn = _sdf->GetElement("is_on")->Get<bool>();

  this->InitSwitchablePlugin(this->sensorTopic_, isSensorOn);
  gzmsg << "CPCROSPlugin initialized!" << std::endl
    << "\t- Input topic= " << inputTopic << std::endl
    << "\t- Output topic= " << this->sensorTopic_ << std::endl;
}

/////////////////////////////////////////////////
void GazeboCPCROSPlugin::OnPlumeParticlesUpdate(
  const sensor_msgs::PointCloud::ConstPtr &_msg)
{
  this->updatingCloud = true;

  double totalParticleConc = 0.0;

  double smoothingParam;
  double particleConc;
  double distToParticle;

  math::Vector3 linkPos = this->link_->GetWorldPose().pos;

  this->outputMsg.is_measuring = true;
  // Store the current position in the local ENU frame where this
  // measurement was taken
  this->outputMsg.position_enu.x = linkPos.x;
  this->outputMsg.position_enu.y = linkPos.y;
  this->outputMsg.position_enu.z = linkPos.z;

  // Calculate the current position in WGS84 spherical coordinates
  ignition::math::Vector3d cartVec = ignition::math::Vector3d(linkPos.x,
    linkPos.y, linkPos.z);

  ignition::math::Vector3d scVec =
    this->link_->GetWorld()->GetSphericalCoordinates()->SphericalFromLocal(cartVec);
  this->outputMsg.latitude = scVec.X();
  this->outputMsg.longitude = scVec.Y();
  this->outputMsg.depth = -1 * scVec.Z();

  // Store this measurement's time stamp
  this->lastUpdateTimestamp = _msg->header.stamp;

  double currentTime = _msg->header.stamp.toSec();

  double initSmoothingLength = std::pow(this->smoothingLength, 2.0 / 3);

  math::Vector3 particle;
  for (int i = 0; i < _msg->points.size(); i++)
  {
    particle = math::Vector3(_msg->points[i].x, _msg->points[i].y,
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

/////////////////////////////////////////////////
bool GazeboCPCROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  bool measurementOK = GazeboCPCPlugin::OnUpdate(_info) && !this->updatingCloud;

  // Publish sensor state
  this->PublishState();

  if (!measurementOK || !this->IsOn())
    return false;

  // Set particle concentration to zero if the point cloud message has not
  // been received for a long time
  if (_info.simTime.Double() - this->lastUpdateTimestamp.toSec() > 1.0)
  {
    this->outputMsg.is_measuring = false;
    this->outputMsg.concentration = 0.0;
  }

  this->outputMsg.concentration += this->noiseAmplitude * this->normal_(this->rndGen_);
  this->outputMsg.header.stamp.sec = _info.simTime.sec;
  this->outputMsg.header.stamp.nsec = _info.simTime.nsec;
  this->measurementPub.publish(this->outputMsg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboCPCROSPlugin)

}
