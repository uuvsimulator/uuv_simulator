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

  this->nedTransform = ignition::math::Pose3d::Zero;
  this->nedTransformIsInit = true;
}

/////////////////////////////////////////////////
PoseGTROSPlugin::~PoseGTROSPlugin()
{ }

/////////////////////////////////////////////////
void PoseGTROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  ignition::math::Vector3d vec;
  GetSDFParam<ignition::math::Vector3d>(_sdf, "position_offset",
    vec, ignition::math::Vector3d::Zero);
  this->offset.Pos() = vec;
  GetSDFParam<ignition::math::Vector3d>(_sdf, "orientation_offset", vec,
    ignition::math::Vector3d::Zero);
  this->offset.Rot() = ignition::math::Quaterniond(vec);

  GetSDFParam<bool>(_sdf, "publish_ned_odom", this->publishNEDOdom, false);

  if (this->publishNEDOdom)
  {
    this->nedFrameID = this->link->GetName() + "_ned";
    this->nedOdomPub = this->rosNode->advertise<nav_msgs::Odometry>(
      this->sensorOutputTopic + "_ned", 1);
    this->nedTransformIsInit = false;
  }

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

  this->tfListener.reset(new tf2_ros::TransformListener(this->tfBuffer));

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

  ignition::math::Pose3d linkPose, refLinkPose;
  ignition::math::Vector3d refLinVel, refAngVel;
  ignition::math::Vector3d linkLinVel, linkAngVel;

  this->UpdateNEDTransform();
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

  // Update the reference frame in case it is given as a Gazebo link and
  // read the reference link's linear and angular velocity vectors
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
    // If no Gazebo link is given as a reference, the linear and angular
    // velocity vectors are set to zero
    refLinVel = ignition::math::Vector3d::Zero;
    refAngVel = ignition::math::Vector3d::Zero;
  }

  // Transform pose and velocity vectors to be represented wrt the
  // reference link provided
  linkLinVel -= refLinVel;
  linkAngVel -= refAngVel;

  // Add noise to the link's linear velocity
  linkLinVel += ignition::math::Vector3d(
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp));

  // Add noise to the link's angular velocity
  linkAngVel += ignition::math::Vector3d(
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp));

  // Publish the odometry message of the base_link wrt Gazebo's ENU
  // inertial reference frame
  this->PublishOdomMessage(curTime, linkPose, linkLinVel, linkAngVel);
  // If the world_ned frame exists (North-East-Down reference frame),
  // the odometry is also published from the robot's base_link_ned wrt
  // world_ned
  this->PublishNEDOdomMessage(curTime, linkPose, linkLinVel, linkAngVel);

  // Store the time stamp for this measurement
  this->lastMeasurementTime = curTime;
  return true;
}

void PoseGTROSPlugin::PublishOdomMessage(common::Time _time,
  ignition::math::Pose3d _pose, ignition::math::Vector3d _linVel,
  ignition::math::Vector3d _angVel)
{
  // Generates the odometry message of the robot's base_link frame wrt
  // Gazebo's default ENU inertial reference frame
  nav_msgs::Odometry odomMsg;

  // Initialize header of the odometry message
  odomMsg.header.frame_id = "world";
  odomMsg.header.stamp.sec = _time.sec;
  odomMsg.header.stamp.nsec = _time.nsec;
  odomMsg.child_frame_id = this->link->GetName();

  // Apply pose offset
  _pose += this->offset;

  // Fill out the messages
  odomMsg.pose.pose.position.x = _pose.Pos().X();
  odomMsg.pose.pose.position.y = _pose.Pos().Y();
  odomMsg.pose.pose.position.z = _pose.Pos().Z();

  odomMsg.pose.pose.orientation.x = _pose.Rot().X();
  odomMsg.pose.pose.orientation.y = _pose.Rot().Y();
  odomMsg.pose.pose.orientation.z = _pose.Rot().Z();
  odomMsg.pose.pose.orientation.w = _pose.Rot().W();

  odomMsg.twist.twist.linear.x = _linVel.X();
  odomMsg.twist.twist.linear.y = _linVel.Y();
  odomMsg.twist.twist.linear.z = _linVel.Z();

  odomMsg.twist.twist.angular.x = _angVel.X();
  odomMsg.twist.twist.angular.y = _angVel.Y();
  odomMsg.twist.twist.angular.z = _angVel.Z();

  // Fill in the covariance matrix
  double gn2 = this->noiseSigma * this->noiseSigma;
  odomMsg.pose.covariance[0] = gn2;
  odomMsg.pose.covariance[7] = gn2;
  odomMsg.pose.covariance[14] = gn2;
  odomMsg.pose.covariance[21] = gn2;
  odomMsg.pose.covariance[28] = gn2;
  odomMsg.pose.covariance[35] = gn2;

  odomMsg.twist.covariance[0] = gn2;
  odomMsg.twist.covariance[7] = gn2;
  odomMsg.twist.covariance[14] = gn2;
  odomMsg.twist.covariance[21] = gn2;
  odomMsg.twist.covariance[28] = gn2;
  odomMsg.twist.covariance[35] = gn2;

  this->rosSensorOutputPub.publish(odomMsg);
}

void PoseGTROSPlugin::PublishNEDOdomMessage(common::Time _time,
  ignition::math::Pose3d _pose, ignition::math::Vector3d _linVel,
  ignition::math::Vector3d _angVel)
{
  // Generates the odometry message of the robot's base_link_ned frame
  // wrt generated NED inertial reference frame
  if (!this->publishNEDOdom)
    return;

  if (!this->nedTransformIsInit)
    return;

  nav_msgs::Odometry odomMsg;

  // Initialize header of the odometry message
  odomMsg.header.frame_id = this->referenceFrameID;
  odomMsg.header.stamp.sec = _time.sec;
  odomMsg.header.stamp.nsec = _time.nsec;
  odomMsg.child_frame_id = this->nedFrameID;

  _pose.Pos() = _pose.Pos() - this->referenceFrame.Pos();
  _pose.Pos() = this->referenceFrame.Rot().RotateVectorReverse(_pose.Pos());

  ignition::math::Quaterniond q = this->nedTransform.Rot();
  q = _pose.Rot() * q;
  q =  this->referenceFrame.Rot() * q;
  _pose.Rot() = q;

  _linVel = this->referenceFrame.Rot().RotateVector(_linVel);
  _angVel = this->referenceFrame.Rot().RotateVector(_angVel);

  // Apply pose offset
  _pose += this->offset;

  // Fill out the messages
  odomMsg.pose.pose.position.x = _pose.Pos().X();
  odomMsg.pose.pose.position.y = _pose.Pos().Y();
  odomMsg.pose.pose.position.z = _pose.Pos().Z();

  odomMsg.pose.pose.orientation.x = _pose.Rot().X();
  odomMsg.pose.pose.orientation.y = _pose.Rot().Y();
  odomMsg.pose.pose.orientation.z = _pose.Rot().Z();
  odomMsg.pose.pose.orientation.w = _pose.Rot().W();

  odomMsg.twist.twist.linear.x = _linVel.X();
  odomMsg.twist.twist.linear.y = _linVel.Y();
  odomMsg.twist.twist.linear.z = _linVel.Z();

  odomMsg.twist.twist.angular.x = _angVel.X();
  odomMsg.twist.twist.angular.y = _angVel.Y();
  odomMsg.twist.twist.angular.z = _angVel.Z();

  double gn2 = this->noiseSigma * this->noiseSigma;
  odomMsg.pose.covariance[0] = gn2;
  odomMsg.pose.covariance[7] = gn2;
  odomMsg.pose.covariance[14] = gn2;
  odomMsg.pose.covariance[21] = gn2;
  odomMsg.pose.covariance[28] = gn2;
  odomMsg.pose.covariance[35] = gn2;

  odomMsg.twist.covariance[0] = gn2;
  odomMsg.twist.covariance[7] = gn2;
  odomMsg.twist.covariance[14] = gn2;
  odomMsg.twist.covariance[21] = gn2;
  odomMsg.twist.covariance[28] = gn2;
  odomMsg.twist.covariance[35] = gn2;

  this->nedOdomPub.publish(odomMsg);
}

/////////////////////////////////////////////////
void PoseGTROSPlugin::UpdateNEDTransform()
{
  if (!this->publishNEDOdom)
    return;
  if (this->nedTransformIsInit)
    return;

  geometry_msgs::TransformStamped childTransform;
  std::string targetFrame = this->nedFrameID;
  std::string sourceFrame = this->link->GetName();
  try
  {
    childTransform = this->tfBuffer.lookupTransform(
      targetFrame, sourceFrame, ros::Time(0));
  }
  catch(tf2::TransformException &ex)
  {
    gzmsg << "Transform between " << targetFrame << " and " << sourceFrame
      << std::endl;
    gzmsg << ex.what() << std::endl;
    return;
  }

  this->nedTransform.Pos() = ignition::math::Vector3d(
    childTransform.transform.translation.x,
    childTransform.transform.translation.y,
    childTransform.transform.translation.z);
  this->nedTransform.Rot() = ignition::math::Quaterniond(
    childTransform.transform.rotation.w,
    childTransform.transform.rotation.x,
    childTransform.transform.rotation.y,
    childTransform.transform.rotation.z);

  this->nedTransformIsInit = true;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(PoseGTROSPlugin)

}
