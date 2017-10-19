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

#include <uuv_sensor_plugins_ros/IMUROSPlugin.hh>

namespace gazebo {

GazeboIMUROSPlugin::GazeboIMUROSPlugin() : GazeboIMUPlugin()
{
}

GazeboIMUROSPlugin::~GazeboIMUROSPlugin()
{
}

void GazeboIMUROSPlugin::Load(gazebo::physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf)
{
    try {
        GazeboIMUPlugin::Load(_parent, _sdf);
    } catch(gazebo::common::Exception &_e)
    {
        gzerr << "Error loading GazeboIMUPlugin" << std::endl;
        return;
    }

    if (!ros::isInitialized())
    {
        gzerr << "Not loading GazeboIMUPlugin since ROS has not been properly "
              << "initialized." << std::endl;
        return;
    }

    this->rosNode.reset(new ros::NodeHandle(this->namespace_));

    this->world_ = _parent->GetWorld();
    this->model_ = _parent;

    this->pubImu_ = this->rosNode->advertise<sensor_msgs::Imu>(
                this->sensorTopic_, 10);

    this->imuRosMessage_.header.frame_id = this->linkName_;

    // We assume uncorrelated noise on the 3 channels -> only set diagonal
    // elements. Only the broadband noise component is considered, specified as
    // a continuous-time density (two-sided spectrum); not the true covariance
    // of the measurements.
    // Angular velocity measurement covariance.
    const ImuParameters& p = this->imuParameters_;
    double gyroVar = p.gyroscopeNoiseDensity*p.gyroscopeNoiseDensity;
    this->imuRosMessage_.angular_velocity_covariance[0] = gyroVar;
    this->imuRosMessage_.angular_velocity_covariance[4] = gyroVar;
    this->imuRosMessage_.angular_velocity_covariance[8] = gyroVar;

    // Linear acceleration measurement covariance.
    double accelVar = p.accelerometerNoiseDensity*p.accelerometerNoiseDensity;
    this->imuRosMessage_.linear_acceleration_covariance[0] = accelVar;
    this->imuRosMessage_.linear_acceleration_covariance[4] = accelVar;
    this->imuRosMessage_.linear_acceleration_covariance[8] = accelVar;

    // Orientation estimate covariance
    double orientationVar = p.orientationNoise*p.orientationNoise;
    this->imuRosMessage_.orientation_covariance[0] = orientationVar;
    this->imuRosMessage_.orientation_covariance[4] = orientationVar;
    this->imuRosMessage_.orientation_covariance[8] = orientationVar;

    bool isSensorOn = true;
    if (_sdf->HasElement("is_on"))
      isSensorOn = _sdf->GetElement("is_on")->Get<bool>();

    this->InitSwitchablePlugin(this->sensorTopic_, isSensorOn);
}

bool GazeboIMUROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    bool measurementOK = GazeboIMUPlugin::OnUpdate(_info);

    // Publish sensor state
    this->PublishState();

    if (!measurementOK || !this->IsOn())
        return false;

    this->imuRosMessage_.header.stamp.sec = _info.simTime.sec;
    this->imuRosMessage_.header.stamp.nsec = _info.simTime.nsec;

    this->imuRosMessage_.orientation.x = this->measOrientation_.x();
    this->imuRosMessage_.orientation.y = this->measOrientation_.y();
    this->imuRosMessage_.orientation.z = this->measOrientation_.z();
    this->imuRosMessage_.orientation.w = this->measOrientation_.w();

    this->imuRosMessage_.linear_acceleration.x = this->measLinearAcc_[0];
    this->imuRosMessage_.linear_acceleration.y = this->measLinearAcc_[1];
    this->imuRosMessage_.linear_acceleration.z = this->measLinearAcc_[2];

    this->imuRosMessage_.angular_velocity.x = this->measAngularVel_[0];
    this->imuRosMessage_.angular_velocity.y = this->measAngularVel_[1];
    this->imuRosMessage_.angular_velocity.z = this->measAngularVel_[2];

    this->pubImu_.publish(this->imuRosMessage_);

    return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboIMUROSPlugin);
}
