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

#include <uuv_sensor_plugins_ros/DVLROSPlugin.hh>

#include <gazebo/physics/Model.hh>

namespace gazebo {

GazeboDVLROSPlugin::GazeboDVLROSPlugin() : GazeboDVLPlugin()
{
}

GazeboDVLROSPlugin::~GazeboDVLROSPlugin()
{
}

void GazeboDVLROSPlugin::Load(gazebo::physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf)
{
  try {
    GazeboDVLPlugin::Load(_parent, _sdf);
  } catch(gazebo::common::Exception &_e)
  {
    gzerr << "Error loading GazeboDVLPlugin" << std::endl;
    return;
  }

  if (!ros::isInitialized())
  {
    gzerr << "Not loading GazeboDVLPlugin since ROS has not been properly "
          << "initialized." << std::endl;
    return;
  }

  this->rosNode.reset(new ros::NodeHandle(this->namespace_));

  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // Prepare constant parts of ros messages
  this->dvlMessage.header.frame_id = this->linkName_;
  this->twistMessage.header.frame_id = this->linkName_;

  double variance = this->velocityNoise*this->velocityNoise;


  for (int i = 0; i < 9; i++)
    this->dvlMessage.velocity_covariance[i] = 0;

  this->dvlMessage.velocity_covariance[0] = variance;
  this->dvlMessage.velocity_covariance[4] = variance;
  this->dvlMessage.velocity_covariance[8] = variance;

  for (int i = 0; i < 36; i++)
    this->twistMessage.twist.covariance[i] = 0.0;

  this->twistMessage.twist.covariance[0] = variance;
  this->twistMessage.twist.covariance[7] = variance;
  this->twistMessage.twist.covariance[14] = variance;
  this->twistMessage.twist.covariance[21] = -1;  // not available
  this->twistMessage.twist.covariance[28] = -1;  // not available
  this->twistMessage.twist.covariance[35] = -1;  // not available

  bool isSensorOn = true;
  if (_sdf->HasElement("is_on"))
    isSensorOn = _sdf->GetElement("is_on")->Get<bool>();

  this->InitSwitchablePlugin(this->sensorTopic_, isSensorOn);

  this->pubDvl = this->rosNode->advertise<uuv_sensor_plugins_ros_msgs::DVL>(
        this->sensorTopic_, 10);

  this->pubTwist = this->rosNode->advertise<
      geometry_msgs::TwistWithCovarianceStamped
      >(this->sensorTopic_ + "_twist", 10);
}

bool GazeboDVLROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  bool measurementOK = GazeboDVLPlugin::OnUpdate(_info);

  // Publish sensor state
  this->PublishState();

  if (!measurementOK || !this->IsOn())
    return false;

  this->dvlMessage.header.stamp.sec = _info.simTime.sec;
  this->dvlMessage.header.stamp.nsec = _info.simTime.nsec;

  this->dvlMessage.velocity.x = this->vel.x;
  this->dvlMessage.velocity.y = this->vel.y;
  this->dvlMessage.velocity.z = this->vel.z;
  this->pubDvl.publish(this->dvlMessage);

  this->twistMessage.header.stamp = this->dvlMessage.header.stamp;

  this->twistMessage.twist.twist.linear.x = this->vel.x;
  this->twistMessage.twist.twist.linear.y = this->vel.y;
  this->twistMessage.twist.twist.linear.z = this->vel.z;

  this->pubTwist.publish(this->twistMessage);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboDVLROSPlugin);
}
