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


#include <uuv_sensor_plugins/IMUPlugin.hh>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <string>

#include <boost/bind.hpp>

#include "Common.hh"

namespace gazebo {

GazeboIMUPlugin::GazeboIMUPlugin()
    : GazeboSensorPlugin()
{
}

GazeboIMUPlugin::~GazeboIMUPlugin()
{
}


void GazeboIMUPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GazeboSensorPlugin::Load(_model, _sdf);

    // Only need to load settings specific to this sensor.
    getSdfParam<double>(_sdf, "gyroscopeNoiseDensity",
                        imuParameters_.gyroscopeNoiseDensity,
                        imuParameters_.gyroscopeNoiseDensity);
    getSdfParam<double>(_sdf, "gyroscopeBiasRandomWalk",
                        imuParameters_.gyroscopeRandomWalk,
                        imuParameters_.gyroscopeRandomWalk);
    getSdfParam<double>(_sdf, "gyroscopeBiasCorrelationTime",
                        imuParameters_.gyroscopeBiasCorrelationTime,
                        imuParameters_.gyroscopeBiasCorrelationTime);
    assert(imuParameters_.gyroscopeBiasCorrelationTime > 0.0);
    getSdfParam<double>(_sdf, "gyroscopeTurnOnBiasSigma",
                        imuParameters_.gyroscopeTurnOnBiasSigma,
                        imuParameters_.gyroscopeTurnOnBiasSigma);
    getSdfParam<double>(_sdf, "accelerometerNoiseDensity",
                        imuParameters_.accelerometerNoiseDensity,
                        imuParameters_.accelerometerNoiseDensity);
    getSdfParam<double>(_sdf, "accelerometerRandomWalk",
                        imuParameters_.accelerometerRandomWalk,
                        imuParameters_.accelerometerRandomWalk);
    getSdfParam<double>(_sdf, "accelerometerBiasCorrelationTime",
                        imuParameters_.accelerometerBiasCorrelationTime,
                        imuParameters_.accelerometerBiasCorrelationTime);
    assert(imuParameters_.accelerometerBiasCorrelationTime > 0.0);
    getSdfParam<double>(_sdf, "accelerometerTurnOnBiasSigma",
                        imuParameters_.accelerometerTurnOnBiasSigma,
                        imuParameters_.accelerometerTurnOnBiasSigma);
    getSdfParam<double>(_sdf, "orientationNoise",
                        imuParameters_.orientationNoise,
                        imuParameters_.orientationNoise);

    if (this->sensorTopic_.empty())
      this->sensorTopic_ = "imu";

    publisher_ = nodeHandle_->Advertise<sensor_msgs::msgs::Imu>(
      this->sensorTopic_, 10);

    // Fill imu message.
    // imu_message_.header.frame_id = frame_id_; TODO Add header
    // We assume uncorrelated noise on the 3 channels -> only set diagonal
    // elements. Only the broadband noise component is considered, specified as
    // a continuous-time density (two-sided spectrum); not the true covariance
    // of the measurements.
    // Angular velocity measurement covariance.
    for (int i = 0; i < 9; i++)
    {
        switch (i)
        {
        case 0:
            imuGazeboMessage_.add_angular_velocity_covariance(
                        imuParameters_.gyroscopeNoiseDensity *
                        imuParameters_.gyroscopeNoiseDensity);

            imuGazeboMessage_.add_orientation_covariance(-1.0);

            imuGazeboMessage_.add_linear_acceleration_covariance(
                        imuParameters_.accelerometerNoiseDensity *
                        imuParameters_.accelerometerNoiseDensity);
            break;
        case 1:
        case 2:
        case 3:
            imuGazeboMessage_.add_angular_velocity_covariance(0.0);

            imuGazeboMessage_.add_orientation_covariance(-1.0);

            imuGazeboMessage_.add_linear_acceleration_covariance(0.0);
            break;
        case 4:
            imuGazeboMessage_.add_angular_velocity_covariance(
                        imuParameters_.gyroscopeNoiseDensity *
                        imuParameters_.gyroscopeNoiseDensity);

            imuGazeboMessage_.add_orientation_covariance(-1.0);

            imuGazeboMessage_.add_linear_acceleration_covariance(
                        imuParameters_.accelerometerNoiseDensity *
                        imuParameters_.accelerometerNoiseDensity);
            break;
        case 5:
        case 6:
        case 7:
            imuGazeboMessage_.add_angular_velocity_covariance(0.0);

            imuGazeboMessage_.add_orientation_covariance(-1.0);

            imuGazeboMessage_.add_linear_acceleration_covariance(0.0);
            break;
        case 8:
            imuGazeboMessage_.add_angular_velocity_covariance(
                        imuParameters_.gyroscopeNoiseDensity *
                        imuParameters_.gyroscopeNoiseDensity);

            imuGazeboMessage_.add_orientation_covariance(-1.0);

            imuGazeboMessage_.add_linear_acceleration_covariance(
                        imuParameters_.accelerometerNoiseDensity *
                        imuParameters_.accelerometerNoiseDensity);
            break;
        }
    }

    gravityW_ = world_->GetPhysicsEngine()->GetGravity();


    double sigma_bon_g = imuParameters_.gyroscopeTurnOnBiasSigma;
    double sigma_bon_a = imuParameters_.accelerometerTurnOnBiasSigma;
    for (int i = 0; i < 3; ++i)
    {
        gyroscopeTurnOnBias_[i] =
                sigma_bon_g * normal_(rndGen_);
        accelerometerTurnOnBias_[i] =
                sigma_bon_a * normal_(rndGen_);
    }

    // TODO(nikolicj) incorporate steady-state covariance of bias process
    gyroscopeBias_.setZero();
    accelerometerBias_.setZero();
}

/// \brief This function adds noise to acceleration and angular rates for
///        accelerometer and gyroscope measurement simulation.
void GazeboIMUPlugin::AddNoise(Eigen::Vector3d &linear_acceleration,
                               Eigen::Vector3d &angular_velocity,
                               Eigen::Quaterniond &orientation,
                               double dt)
{
    // CHECK(linear_acceleration);
    // CHECK(angular_velocity);
    assert(dt > 0.0);

    /// Gyrosocpe
    double tau_g = imuParameters_.gyroscopeBiasCorrelationTime;
    // Discrete-time standard deviation equivalent to an "integrating" sampler
    // with integration time dt.
    double sigma_g_d = 1 / sqrt(dt) * imuParameters_.gyroscopeNoiseDensity;
    double sigma_b_g = imuParameters_.gyroscopeRandomWalk;
    // Compute exact covariance of the process after dt [Maybeck 4-114].
    double sigma_b_g_d =
            sqrt(- sigma_b_g * sigma_b_g * tau_g / 2.0 *
                  (exp(-2.0 * dt / tau_g) - 1.0));
    // Compute state-transition.
    double phi_g_d = exp(-1.0 / tau_g * dt);

    // Simulate gyroscope noise processes and add them to the true angular rate.
    for (int i = 0; i < 3; ++i)
    {
        gyroscopeBias_[i] = phi_g_d * gyroscopeBias_[i] +
                sigma_b_g_d * normal_(rndGen_);
        angular_velocity[i] = angular_velocity[i] +
                gyroscopeBias_[i] +
                sigma_g_d * normal_(rndGen_) +
                gyroscopeTurnOnBias_[i];
    }

    /// Accelerometer
    double tau_a = imuParameters_.accelerometerBiasCorrelationTime;
    // Discrete-time standard deviation equivalent to an "integrating" sampler
    // with integration time dt.
    double sigma_a_d = 1/sqrt(dt) * imuParameters_.accelerometerNoiseDensity;
    double sigma_b_a = imuParameters_.accelerometerRandomWalk;
    // Compute exact covariance of the process after dt [Maybeck 4-114].
    double sigma_b_a_d =
            sqrt(- sigma_b_a * sigma_b_a * tau_a / 2.0 *
                  (exp(-2.0 * dt / tau_a) - 1.0));
    // Compute state-transition.
    double phi_a_d = exp(-1.0 / tau_a * dt);

    // Simulate accelerometer noise processes and add them to the true linear
    // acceleration.
    for (int i = 0; i < 3; ++i)
    {
        accelerometerBias_[i] = phi_a_d * accelerometerBias_[i] +
                sigma_b_a_d * normal_(rndGen_);
        linear_acceleration[i] +=  accelerometerBias_[i] +
                                   sigma_a_d * normal_(rndGen_) +
                                   accelerometerTurnOnBias_[i];
    }

    /// Orientation
    // Construct error quaterion using small-angle approximation.
    double scale = 0.5*imuParameters_.orientationNoise;
    // Attention: w-xyz
    Eigen::Quaterniond error(1.0, scale*normal_(rndGen_),
                             scale*normal_(rndGen_), scale*normal_(rndGen_));
    error.normalize();
    orientation = orientation*error;
}

void GazeboIMUPlugin::SimulateMeasurement(const common::UpdateInfo& _info)
{
    common::Time current_time  = _info.simTime;
    double dt = (current_time - lastMeasTime_).Double();

    lastMeasTime_ = current_time;
    double t = current_time.Double();

    math::Pose T_W_I = link_->GetWorldPose();  // TODO(burrimi): Check tf.
    math::Quaternion C_W_I = T_W_I.rot;

#if GAZEBO_MAJOR_VERSION < 5
    math::Vector3 velocity_current_W = link_->GetWorldLinearVel();
    // link_->GetRelativeLinearAccel() does not work sometimes with old gazebo.
    // TODO For an accurate simulation, this might have to be fixed.
    // This issue is solved in gazebo 5.
    math::Vector3 acceleration = (velocity_current_W - velocity_prev_W_) / dt;
    math::Vector3 acceleration_I =
            C_W_I.RotateVectorReverse(acceleration - gravity_W_);

    velocity_prev_W_ = velocity_current_W;
#else
    math::Vector3 acceleration_I = link_->GetRelativeLinearAccel()
            - C_W_I.RotateVectorReverse(gravityW_);
#endif

    math::Vector3 angular_vel_I = link_->GetRelativeAngularVel();

    // Warning: This constructor expects w first, even though eigen internally
    //          uses xyzw.
    this->measOrientation_ = Eigen::Quaterniond(C_W_I.w, C_W_I.x,
                                                C_W_I.y, C_W_I.z);

    this->measLinearAcc_ = Eigen::Vector3d(acceleration_I.x,
                                          acceleration_I.y,
                                          acceleration_I.z);

    this->measAngularVel_ = Eigen::Vector3d(angular_vel_I.x,
                                            angular_vel_I.y,
                                            angular_vel_I.z);

    AddNoise(this->measLinearAcc_, this->measAngularVel_,
             this->measOrientation_, dt);
}

bool GazeboIMUPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    if (!ShouldIGenerate(_info))
        return false;

    SimulateMeasurement(_info);

    // Copy math::Quaternion to gazebo::msgs::Quaternion
    gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
    orientation->set_x(this->measOrientation_.x());
    orientation->set_y(this->measOrientation_.y());
    orientation->set_z(this->measOrientation_.z());
    orientation->set_w(this->measOrientation_.w());

    // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
    gazebo::msgs::Vector3d* linear_acceleration = new gazebo::msgs::Vector3d();
    linear_acceleration->set_x(this->measLinearAcc_[0]);
    linear_acceleration->set_y(this->measLinearAcc_[1]);
    linear_acceleration->set_z(this->measLinearAcc_[2]);

    // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
    gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
    angular_velocity->set_x(this->measAngularVel_[0]);
    angular_velocity->set_y(this->measAngularVel_[1]);
    angular_velocity->set_z(this->measAngularVel_[2]);

    imuGazeboMessage_.set_allocated_orientation(orientation);
    imuGazeboMessage_.set_allocated_linear_acceleration(linear_acceleration);
    imuGazeboMessage_.set_allocated_angular_velocity(angular_velocity);

    publisher_->Publish(imuGazeboMessage_);

    return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboIMUPlugin);
}
