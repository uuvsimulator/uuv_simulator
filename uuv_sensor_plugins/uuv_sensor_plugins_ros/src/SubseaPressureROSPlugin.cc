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

#include <uuv_sensor_plugins_ros/SubseaPressureROSPlugin.hh>

#include <gazebo/physics/physics.hh>

namespace gazebo {

GazeboSubseaPressureROSPlugin::GazeboSubseaPressureROSPlugin()
    : GazeboSubseaPressurePlugin()
{
}

GazeboSubseaPressureROSPlugin::~GazeboSubseaPressureROSPlugin()
{
}

void GazeboSubseaPressureROSPlugin::Load(gazebo::physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf)
{
    try {
        GazeboSubseaPressurePlugin::Load(_parent, _sdf);
    } catch(gazebo::common::Exception &_e)
    {
        gzerr << "Error loading GazeboSubseaPressurePlugin" << std::endl;
        return;
    }

    if (!ros::isInitialized())
    {
        gzerr << "Not loading GazeboSubseaPressureROSPlugin since ROS has not "
              << "been properly initialized." << std::endl;
        return;
    }

    this->rosNode.reset(new ros::NodeHandle(this->namespace_));

    this->world_ = _parent->GetWorld();
    this->model_ = _parent;

    this->rosPublisher_ = this->rosNode->advertise<sensor_msgs::FluidPressure>(
                this->sensorTopic_, 10);

    bool isSensorOn = true;
    if (_sdf->HasElement("is_on"))
      isSensorOn = _sdf->GetElement("is_on")->Get<bool>();

    this->InitSwitchablePlugin(this->sensorTopic_, isSensorOn);
}

bool GazeboSubseaPressureROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    bool measurementOK = GazeboSubseaPressurePlugin::OnUpdate(_info);

    // Publish sensor state
    this->PublishState();

    if (!measurementOK || !this->IsOn())
        return false;

    sensor_msgs::FluidPressure msg;

    msg.header.stamp.sec  = _info.simTime.sec;
    msg.header.stamp.nsec = _info.simTime.nsec;
    msg.header.frame_id = this->linkName_;

    msg.fluid_pressure = this->measuredPressure_;
    msg.variance = this->stdDev_*this->stdDev_;

    this->rosPublisher_.publish(msg);

    return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboSubseaPressureROSPlugin);
}
