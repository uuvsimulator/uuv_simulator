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

// This source code is derived from gazebo_ros_pkgs
//   (https://github.com/ros-simulation/gazebo_ros_pkgs)
// * Copyright 2012 Open Source Robotics Foundation,
// licensed under the Apache-2.0 license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// The original code was modified to:
// - be more consistent with other sensor plugins within uuv_simulator,
// - adhere to Gazebo's coding standards.

#include <uuv_sensor_ros_plugins/PoseGTROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
PoseGTROSPlugin::PoseGTROSPlugin() : ROSBaseModelPlugin()
{
  this->offset.Pos() = ignition::math::Vector3d::Zero;
  this->offset.Rot() = ignition::math::Quaterniond(
    ignition::math::Vector3d(0, 0, 0));

  // Initialize the reference's velocity and acceleration vectors
  this->refLinAcc = ignition::math::Vector3d::Zero;
  this->refAngAcc = ignition::math::Vector3d::Zero;
}

/////////////////////////////////////////////////
PoseGTROSPlugin::~PoseGTROSPlugin()
{ }

/////////////////////////////////////////////////
void PoseGTROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  GetSDFParam<double>(_sdf, "gaussian_noise_sigma", this->gaussianNoiseSigma,
    0.0);

  ignition::math::Vector3d vec;
  GetSDFParam<ignition::math::Vector3d>(_sdf, "position_offset",
    vec, ignition::math::Vector3d::Zero);
  this->offset.Pos() = vec;
  GetSDFParam<ignition::math::Vector3d>(_sdf, "orientation_offset", vec,
    ignition::math::Vector3d::Zero);
  this->offset.Rot() = ignition::math::Quaterniond(vec);

  // Initialize the reference's velocity and acceleration vectors
  if (!this->referenceLink)
  {
    this->lastRefLinVel = ignition::math::Vector3d::Zero;
    this->lastRefAngVel = ignition::math::Vector3d::Zero;
  }
  else
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->lastRefLinVel = this->referenceLink->WorldLinearVel();
    this->lastRefAngVel = this->referenceLink->WorldAngularVel();
#else
    this->lastRefLinVel = this->referenceLink->GetWorldLinearVel().Ign();
    this->lastRefAngVel = this->referenceLink->GetWorldAngularVel().Ign();
#endif
  }

  this->rosSensorOutputPub = this->rosNode->advertise<nav_msgs::Odometry>(
      this->sensorOutputTopic, 1);
}

/////////////////////////////////////////////////
bool PoseGTROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  if (!this->EnableMeasurement(_info))
    return false;

    // Read the current simulation time
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time curTime = this->world->SimTime();
#else
    common::Time curTime = this->world->GetSimTime();
#endif

  double dt = curTime.Double() - this->lastMeasurementTime.Double();

  if (dt <= 0)
    return false;

  // Initialize header of the odometry message
  this->odomMsg.header.frame_id = this->referenceFrameID;
  this->odomMsg.header.stamp.sec = curTime.sec;
  this->odomMsg.header.stamp.nsec = curTime.nsec;
  this->odomMsg.child_frame_id = this->link->GetName();

  ignition::math::Pose3d linkPose, refLinkPose;
  ignition::math::Vector3d refLinVel, refAngVel;
  ignition::math::Vector3d linkLinVel, linkAngVel;

    // Read sensor link's current pose and velocity
#if GAZEBO_MAJOR_VERSION >= 8
  linkLinVel = this->link->WorldLinearVel();
  linkAngVel = this->link->WorldAngularVel();

  linkPose = this->link->WorldPose();
#else
  linkLinVel = this->link->GetWorldLinearVel().Ign();
  linkAngVel = this->link->GetWorldAngularVel().Ign();

  linkPose = this->link->GetWorldPose().Ign();
#endif

  this->UpdateReferenceFramePose();
  if (this->referenceLink)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    refLinVel = this->referenceLink->WorldLinearVel();
    refAngVel = this->referenceLink->WorldAngularVel();

    this->referenceFrame = this->referenceLink->WorldPose();
#else
    refLinVel = this->referenceLink->GetWorldLinearVel().Ign();
    refAngVel = this->referenceLink->GetWorldAngularVel().Ign();

    this->referenceFrame = this->referenceLink->GetWorldPose().Ign();
#endif
  }
  else
  {
    refLinVel = ignition::math::Vector3d::Zero;
    refAngVel = ignition::math::Vector3d::Zero;
  }

  // Transform pose and velocity vectors to be represented wrt the
  // reference link provided
  linkPose.Pos() = linkPose.Pos() - this->referenceFrame.Pos();
  linkPose.Pos() = this->referenceFrame.Rot().RotateVectorReverse(linkPose.Pos());
  linkPose.Rot() *= this->referenceFrame.Rot().Inverse();

  linkLinVel = this->referenceFrame.Rot().RotateVector(linkLinVel - refLinVel);
  linkAngVel = this->referenceFrame.Rot().RotateVector(linkAngVel - refAngVel);

  // Apply pose offset
  linkPose.Pos() = linkPose.Pos() + this->offset.Pos();
  linkPose.Rot() = this->offset.Rot() * linkPose.Rot();
  linkPose.Rot().Normalize();

  // Fill out the messages
  this->odomMsg.pose.pose.position.x = linkPose.Pos().X();
  this->odomMsg.pose.pose.position.y = linkPose.Pos().Y();
  this->odomMsg.pose.pose.position.z = linkPose.Pos().Z();

  this->odomMsg.pose.pose.orientation.x = linkPose.Rot().X();
  this->odomMsg.pose.pose.orientation.y = linkPose.Rot().Y();
  this->odomMsg.pose.pose.orientation.z = linkPose.Rot().Z();
  this->odomMsg.pose.pose.orientation.w = linkPose.Rot().W();

  this->odomMsg.twist.twist.angular.x = linkLinVel.X() +
    this->GetGaussianNoise(this->gaussianNoiseSigma);
  this->odomMsg.twist.twist.angular.y = linkLinVel.Y() +
    this->GetGaussianNoise(this->gaussianNoiseSigma);
  this->odomMsg.twist.twist.angular.z = linkLinVel.Z() +
    this->GetGaussianNoise(this->gaussianNoiseSigma);

  this->odomMsg.twist.twist.angular.x = linkAngVel.X() +
    this->GetGaussianNoise(this->gaussianNoiseSigma);
  this->odomMsg.twist.twist.angular.y = linkAngVel.Y() +
    this->GetGaussianNoise(this->gaussianNoiseSigma);
  this->odomMsg.twist.twist.angular.z = linkAngVel.Z() +
    this->GetGaussianNoise(this->gaussianNoiseSigma);

  // Fill in the covariance matrix
  double gn2 = this->gaussianNoiseSigma * this->gaussianNoiseSigma;
  this->odomMsg.pose.covariance[0] = gn2;
  this->odomMsg.pose.covariance[7] = gn2;
  this->odomMsg.pose.covariance[14] = gn2;
  this->odomMsg.pose.covariance[21] = gn2;
  this->odomMsg.pose.covariance[28] = gn2;
  this->odomMsg.pose.covariance[35] = gn2;

  this->odomMsg.twist.covariance[0] = gn2;
  this->odomMsg.twist.covariance[7] = gn2;
  this->odomMsg.twist.covariance[14] = gn2;
  this->odomMsg.twist.covariance[21] = gn2;
  this->odomMsg.twist.covariance[28] = gn2;
  this->odomMsg.twist.covariance[35] = gn2;

  this->rosSensorOutputPub.publish(this->odomMsg);
  this->lastMeasurementTime = curTime;
  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(PoseGTROSPlugin)

}
