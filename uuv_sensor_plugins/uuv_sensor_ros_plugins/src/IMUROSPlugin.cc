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
// - adhere to Gazebo's coding standards.

#include <uuv_sensor_ros_plugins/IMUROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
IMUROSPlugin::IMUROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
IMUROSPlugin::~IMUROSPlugin()
{ }

/////////////////////////////////////////////////
void IMUROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  // Only need to load settings specific to this sensor.
  GetSDFParam<double>(_sdf, "gyroscope_noise_density",
                      this->imuParameters.gyroscopeNoiseDensity,
                      this->imuParameters.gyroscopeNoiseDensity);
  GetSDFParam<double>(_sdf, "gyroscope_bias_random_walk",
                      this->imuParameters.gyroscopeRandomWalk,
                      this->imuParameters.gyroscopeRandomWalk);
  GetSDFParam<double>(_sdf, "gyroscope_bias_correlation_time",
                      this->imuParameters.gyroscopeBiasCorrelationTime,
                      this->imuParameters.gyroscopeBiasCorrelationTime);
  GZ_ASSERT(this->imuParameters.gyroscopeBiasCorrelationTime > 0.0,
    "Gyroscope bias correlation time must be greater than zero");
  GetSDFParam<double>(_sdf, "gyroscope_turn_on_bias_sigma",
                      this->imuParameters.gyroscopeTurnOnBiasSigma,
                      this->imuParameters.gyroscopeTurnOnBiasSigma);
  GetSDFParam<double>(_sdf, "accelerometer_noise_density",
                      this->imuParameters.accelerometerNoiseDensity,
                      this->imuParameters.accelerometerNoiseDensity);
  GetSDFParam<double>(_sdf, "accelerometer_random_walk",
                      this->imuParameters.accelerometerRandomWalk,
                      this->imuParameters.accelerometerRandomWalk);
  GetSDFParam<double>(_sdf, "accelerometer_bias_correlation_time",
                      this->imuParameters.accelerometerBiasCorrelationTime,
                      this->imuParameters.accelerometerBiasCorrelationTime);
  GZ_ASSERT(this->imuParameters.accelerometerBiasCorrelationTime > 0.0,
    "Accelerometer bias correlation time must be greater than zero");
  GetSDFParam<double>(_sdf, "accelerometer_turn_on_bias_sigma",
                      this->imuParameters.accelerometerTurnOnBiasSigma,
                      this->imuParameters.accelerometerTurnOnBiasSigma);
  GetSDFParam<double>(_sdf, "orientation_noise",
                      this->imuParameters.orientationNoise,
                      this->imuParameters.orientationNoise);

  this->imuROSMessage.header.frame_id = this->link->GetName();
  // Fill IMU message.
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as
  // a continuous-time density (two-sided spectrum); not the true covariance
  // of the measurements.
  // Angular velocity measurement covariance.
  this->AddNoiseModel("gyro_noise_density", this->imuParameters.gyroscopeNoiseDensity);
  double gyroVar = this->imuParameters.gyroscopeNoiseDensity *
    this->imuParameters.gyroscopeNoiseDensity;
  this->imuROSMessage.angular_velocity_covariance[0] = gyroVar;
  this->imuROSMessage.angular_velocity_covariance[4] = gyroVar;
  this->imuROSMessage.angular_velocity_covariance[8] = gyroVar;

  // Linear acceleration measurement covariance.
  this->AddNoiseModel("acc_noise_density", this->imuParameters.accelerometerNoiseDensity);
  double accelVar = this->imuParameters.accelerometerNoiseDensity *
    this->imuParameters.accelerometerNoiseDensity;
  this->imuROSMessage.linear_acceleration_covariance[0] = accelVar;
  this->imuROSMessage.linear_acceleration_covariance[4] = accelVar;
  this->imuROSMessage.linear_acceleration_covariance[8] = accelVar;

  // Orientation estimate covariance
  this->AddNoiseModel("orientation_noise_density", this->imuParameters.orientationNoise);
  double orientationVar = this->imuParameters.orientationNoise *
    this->imuParameters.orientationNoise;
  this->imuROSMessage.orientation_covariance[0] = orientationVar;
  this->imuROSMessage.orientation_covariance[4] = orientationVar;
  this->imuROSMessage.orientation_covariance[8] = orientationVar;

  // Store the acc. gravity vector
#if GAZEBO_MAJOR_VERSION >= 8
  this->gravityWorld = this->world->Gravity();
#else
  this->gravityWorld = this->world->GetPhysicsEngine()->GetGravity().Ign();
#endif

  double sigmaBonG = this->imuParameters.gyroscopeTurnOnBiasSigma;
  double sigmaBonA = this->imuParameters.accelerometerTurnOnBiasSigma;

  this->AddNoiseModel("gyro_turn_on_bias", sigmaBonG);
  this->AddNoiseModel("acc_turn_on_bias", sigmaBonA);

  // FIXME Add the noise amplitude input for gyroscope
  this->gyroscopeTurnOnBias = ignition::math::Vector3d(
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp));
  // FIXME Add the noise amplitude input for accelerometer
  this->accelerometerTurnOnBias = ignition::math::Vector3d(
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp));

  // TODO(nikolicj) incorporate steady-state covariance of bias process
  this->gyroscopeBias = ignition::math::Vector3d::Zero;
  this->accelerometerBias = ignition::math::Vector3d::Zero;

  this->rosSensorOutputPub =
    this->rosNode->advertise<sensor_msgs::Imu>(this->sensorOutputTopic, 1);

  if (this->gazeboMsgEnabled)
  {
    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<sensor_msgs::msgs::Imu>(
        this->robotNamespace + "/" + this->sensorOutputTopic, 1);
  }
}

/////////////////////////////////////////////////
bool IMUROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  if (this->enableLocalNEDFrame)
    this->SendLocalNEDTransform();

    // Read the current simulation time
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time curTime = this->world->SimTime();
#else
    common::Time curTime = this->world->GetSimTime();
#endif

  double dt = curTime.Double() - this->lastMeasurementTime.Double();

  ignition::math::Pose3d worldLinkPose;
  ignition::math::Vector3d refLinVel;
  ignition::math::Vector3d bodyAngVel;
  ignition::math::Vector3d bodyLinAcc, worldLinAcc;
  ignition::math::Vector3d refGravityWorld;

  // Read sensor link's current pose and velocity
#if GAZEBO_MAJOR_VERSION >= 8
  bodyAngVel = this->link->RelativeAngularVel();
  bodyLinAcc = this->link->RelativeLinearAccel();
  worldLinkPose = this->link->WorldPose();
#else
  bodyAngVel = this->link->GetRelativeAngularVel().Ign();
  bodyLinAcc = this->link->GetRelativeLinearAccel().Ign();
  worldLinkPose = this->link->GetWorldPose().Ign();
#endif

  this->UpdateReferenceFramePose();
  if (this->referenceLink)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->referenceFrame = this->referenceLink->WorldPose();
#else
    this->referenceFrame = this->referenceLink->GetWorldPose().Ign();
#endif
  }

  // Transform pose and velocity vectors to be represented wrt the
  // reference link provided
  worldLinkPose.Pos() = worldLinkPose.Pos() - this->referenceFrame.Pos();
  worldLinkPose.Pos() = this->referenceFrame.Rot().RotateVectorReverse(
    worldLinkPose.Pos());
  worldLinkPose.Rot() *= this->referenceFrame.Rot().Inverse();

  ignition::math::Vector3d gravityBody =
    worldLinkPose.Rot().RotateVectorReverse(this->gravityWorld);

  if (this->enableLocalNEDFrame)
  {
    bodyAngVel = this->localNEDFrame.Rot().RotateVector(bodyAngVel);
    bodyLinAcc = this->localNEDFrame.Rot().RotateVector(bodyLinAcc);
    gravityBody = this->localNEDFrame.Rot().RotateVector(gravityBody);
  }

  // Compute the simulated measurements wrt the default world ENU frame
  this->measLinearAcc = bodyLinAcc - gravityBody;
  this->measAngularVel = bodyAngVel;
  this->measOrientation = worldLinkPose.Rot();

  // Add noise and bias to the simulated data
  this->AddNoise(this->measLinearAcc, this->measAngularVel,
    this->measOrientation, dt);

  // Fill the ROS IMU message
  this->imuROSMessage.header.stamp.sec = _info.simTime.sec;
  this->imuROSMessage.header.stamp.nsec = _info.simTime.nsec;

  this->imuROSMessage.orientation.x = this->measOrientation.X();
  this->imuROSMessage.orientation.y = this->measOrientation.Y();
  this->imuROSMessage.orientation.z = this->measOrientation.Z();
  this->imuROSMessage.orientation.w = this->measOrientation.W();

  this->imuROSMessage.linear_acceleration.x = this->measLinearAcc.X();
  this->imuROSMessage.linear_acceleration.y = this->measLinearAcc.Y();
  this->imuROSMessage.linear_acceleration.z = this->measLinearAcc.Z();

  this->imuROSMessage.angular_velocity.x = this->measAngularVel.X();
  this->imuROSMessage.angular_velocity.y = this->measAngularVel.Y();
  this->imuROSMessage.angular_velocity.z = this->measAngularVel.Z();

  this->rosSensorOutputPub.publish(this->imuROSMessage);

  if (this->gazeboMsgEnabled)
  {
    sensor_msgs::msgs::Imu imuGazeboMessage;
    // Fill the Gazebo IMU message
    for (int i = 0; i < 9; i++)
    {
      switch (i)
      {
        case 0:
          imuGazeboMessage.add_angular_velocity_covariance(
                      this->imuParameters.gyroscopeNoiseDensity *
                      this->imuParameters.gyroscopeNoiseDensity);
          imuGazeboMessage.add_orientation_covariance(-1.0);
          imuGazeboMessage.add_linear_acceleration_covariance(
                      this->imuParameters.accelerometerNoiseDensity *
                      this->imuParameters.accelerometerNoiseDensity);
          break;
        case 1:
        case 2:
        case 3:
          imuGazeboMessage.add_angular_velocity_covariance(0.0);
          imuGazeboMessage.add_orientation_covariance(-1.0);
          imuGazeboMessage.add_linear_acceleration_covariance(0.0);
          break;
        case 4:
          imuGazeboMessage.add_angular_velocity_covariance(
                      this->imuParameters.gyroscopeNoiseDensity *
                      this->imuParameters.gyroscopeNoiseDensity);
          imuGazeboMessage.add_orientation_covariance(-1.0);
          imuGazeboMessage.add_linear_acceleration_covariance(
                      this->imuParameters.accelerometerNoiseDensity *
                      this->imuParameters.accelerometerNoiseDensity);
          break;
        case 5:
        case 6:
        case 7:
          imuGazeboMessage.add_angular_velocity_covariance(0.0);
          imuGazeboMessage.add_orientation_covariance(-1.0);
          imuGazeboMessage.add_linear_acceleration_covariance(0.0);
          break;
        case 8:
          imuGazeboMessage.add_angular_velocity_covariance(
                      this->imuParameters.gyroscopeNoiseDensity *
                      this->imuParameters.gyroscopeNoiseDensity);
          imuGazeboMessage.add_orientation_covariance(-1.0);
          imuGazeboMessage.add_linear_acceleration_covariance(
                      this->imuParameters.accelerometerNoiseDensity *
                      this->imuParameters.accelerometerNoiseDensity);
          break;
      }
    }
    // Copy math::Quaternion to gazebo::msgs::Quaternion
    gazebo::msgs::Quaternion * orientation = new gazebo::msgs::Quaternion();
    orientation->set_x(this->measOrientation.X());
    orientation->set_y(this->measOrientation.Y());
    orientation->set_z(this->measOrientation.Z());
    orientation->set_w(this->measOrientation.W());

    // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
     gazebo::msgs::Vector3d * linAcc = new gazebo::msgs::Vector3d();
    linAcc->set_x(this->measLinearAcc.X());
    linAcc->set_y(this->measLinearAcc.Y());
    linAcc->set_z(this->measLinearAcc.Z());

    // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
    gazebo::msgs::Vector3d * angVel = new gazebo::msgs::Vector3d();
    angVel->set_x(this->measAngularVel.X());
    angVel->set_y(this->measAngularVel.Y());
    angVel->set_z(this->measAngularVel.Z());

    imuGazeboMessage.set_allocated_orientation(orientation);
    imuGazeboMessage.set_allocated_linear_acceleration(linAcc);
    imuGazeboMessage.set_allocated_angular_velocity(angVel);
    this->gazeboSensorOutputPub->Publish(imuGazeboMessage);
  }

  this->lastMeasurementTime = curTime;
  return true;
}

/////////////////////////////////////////////////
void IMUROSPlugin::AddNoise(ignition::math::Vector3d& _linAcc,
  ignition::math::Vector3d& _angVel, ignition::math::Quaterniond& _orientation,
  double _dt)
{
  GZ_ASSERT(_dt > 0.0, "Invalid time step");

  /// Gyroscope
  double tauG = this->imuParameters.gyroscopeBiasCorrelationTime;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigmaGD = 1 / sqrt(_dt) * this->imuParameters.gyroscopeNoiseDensity;
  double sigmaBG = this->imuParameters.gyroscopeRandomWalk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigmaBGD = sqrt(- sigmaBG * sigmaBG * tauG / 2.0 *
    (exp(-2.0 * _dt / tauG) - 1.0));
  // Compute state-transition.
  double phiGD = exp(-1.0 / tauG * _dt);

  // FIXME Add the noise amplitude input for BGD
  this->AddNoiseModel("bgd", sigmaBGD);
  // FIXME Add the noise amplitude input for GD
  this->AddNoiseModel("gd", sigmaGD);

  // Simulate gyroscope noise processes and add them to the true angular rate.
  this->gyroscopeBias = phiGD * this->gyroscopeBias +
    ignition::math::Vector3d(
      this->GetGaussianNoise("bgd", this->noiseAmp),
      this->GetGaussianNoise("bgd", this->noiseAmp),
      this->GetGaussianNoise("bgd", this->noiseAmp));
  _angVel = _angVel + this->gyroscopeBias + this->gyroscopeTurnOnBias +
    ignition::math::Vector3d(
      this->GetGaussianNoise("gd", this->noiseAmp),
      this->GetGaussianNoise("gd", this->noiseAmp),
      this->GetGaussianNoise("gd", this->noiseAmp));

  /// Accelerometer
  double tauA = this->imuParameters.accelerometerBiasCorrelationTime;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigmaAD = 1. / sqrt(_dt) * this->imuParameters.accelerometerNoiseDensity;
  double sigmaBA = this->imuParameters.accelerometerRandomWalk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigmaBAD = sqrt(- sigmaBA * sigmaBA * tauA / 2.0 *
    (exp(-2.0 * _dt / tauA) - 1.0));
  // Compute state-transition.
  double phiAD = exp(-1.0 / tauA * _dt);

  // FIXME Add the noise amplitude input for BAD
  this->AddNoiseModel("bad", sigmaBAD);
  // FIXME Add the noise amplitude input for BAD
  this->AddNoiseModel("ad", sigmaAD);

  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  this->accelerometerBias = phiAD * this->accelerometerBias +
    ignition::math::Vector3d(
      this->GetGaussianNoise("bad", this->noiseAmp),
      this->GetGaussianNoise("bad", this->noiseAmp),
      this->GetGaussianNoise("bad", this->noiseAmp));
  _linAcc = _linAcc + this->accelerometerBias + this->accelerometerTurnOnBias +
    ignition::math::Vector3d(
      this->GetGaussianNoise("ad", this->noiseAmp),
      this->GetGaussianNoise("ad", this->noiseAmp),
      this->GetGaussianNoise("ad", this->noiseAmp));

  /// Orientation
  // Construct error quaterion using small-angle approximation.
  double scale = 0.5 * this->imuParameters.orientationNoise;

  // Attention: w-xyz
  ignition::math::Quaterniond error(1.0,
    this->GetGaussianNoise("orientation_noise_density", scale),
    this->GetGaussianNoise("orientation_noise_density", scale),
    this->GetGaussianNoise("orientation_noise_density", scale));

  error.Normalize();
  _orientation = _orientation * error;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(IMUROSPlugin)
}
