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

#include <uuv_sensor_plugins_ros/MagnetometerRosPlugin.hh>

#include <gazebo/physics/physics.hh>

namespace gazebo {

GazeboMagnetometerRosPlugin::GazeboMagnetometerRosPlugin()
    : GazeboMagnetometerPlugin()
{
}

GazeboMagnetometerRosPlugin::~GazeboMagnetometerRosPlugin()
{
}

void GazeboMagnetometerRosPlugin::Load(gazebo::physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf)
{
    try {
        GazeboMagnetometerPlugin::Load(_parent, _sdf);
    } catch(gazebo::common::Exception &_e)
    {
        gzerr << "Error loading GazeboMagnetometerRosPlugin" << std::endl;
        return;
    }

    if (!ros::isInitialized())
    {
        gzerr << "Not loading GazeboMagnetometerRosPlugin since ROS has not "
              << "been properly initialized." << std::endl;
        return;
    }

    this->rosNode_.reset(new ros::NodeHandle(this->namespace_));

    this->world_ = _parent->GetWorld();
    this->model_ = _parent;

    this->rosPublisher_ = this->rosNode_->advertise<sensor_msgs::MagneticField>(
                this->sensorTopic_, 10);

    // Prepare ROS message
    this->rosMessage_.header.frame_id = this->linkName_;

    this->rosMessage_.magnetic_field_covariance[0] =
            this->parameters_.noiseXY*this->parameters_.noiseXY;
    this->rosMessage_.magnetic_field_covariance[4] =
            this->parameters_.noiseXY*this->parameters_.noiseXY;
    this->rosMessage_.magnetic_field_covariance[8] =
            this->parameters_.noiseZ*this->parameters_.noiseZ;
}

bool GazeboMagnetometerRosPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    bool measurementOK = GazeboMagnetometerPlugin::OnUpdate(_info);

    if (!measurementOK)
        return false;

    this->rosMessage_.header.stamp.sec = _info.simTime.sec;
    this->rosMessage_.header.stamp.nsec = _info.simTime.nsec;

    this->rosMessage_.magnetic_field.x = this->measMagneticField_.x;
    this->rosMessage_.magnetic_field.y = this->measMagneticField_.y;
    this->rosMessage_.magnetic_field.z = this->measMagneticField_.z;

    this->rosPublisher_.publish(this->rosMessage_);

    return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMagnetometerRosPlugin);
}
