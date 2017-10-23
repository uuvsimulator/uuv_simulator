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

#include <uuv_sensor_plugins_ros/RPTROSPlugin.hh>

#include <gazebo/physics/Model.hh>

namespace gazebo {

GazeboRptRosPlugin::GazeboRptRosPlugin() : GazeboRPTPlugin()
{
}

GazeboRptRosPlugin::~GazeboRptRosPlugin()
{
}

void GazeboRptRosPlugin::Load(gazebo::physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf)
{
  try {
    GazeboRPTPlugin::Load(_parent, _sdf);
  } catch(gazebo::common::Exception &_e)
  {
    gzerr << "Error loading GazeboRPTPlugin" << std::endl;
    return;
  }

  if (!ros::isInitialized())
  {
    gzerr << "Not loading GazeboRPTPlugin since ROS has not been properly "
          << "initialized." << std::endl;
    return;
  }

  this->referenceFame = "ssbl";
  if (_sdf->HasElement("referenceFrame"))
  {
    this->referenceFame =
        _sdf->GetElement("referenceFrame")->Get<std::string>();
  }

  this->rosNode.reset(new ros::NodeHandle(this->namespace_));

  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // Prepare constant parts of pos message
  this->posMsg.header.frame_id = this->linkName_;

  double variance = this->positionNoise*this->positionNoise;

  for (int i = 0; i < 9; i++)
    this->posMsg.pos.covariance[i] = 0;

  this->posMsg.pos.covariance[0] = this->posMsg.pos.covariance[4] =
      this->posMsg.pos.covariance[8] = variance;

  this->pubPos = this->rosNode->advertise<
      uuv_sensor_plugins_ros_msgs::PositionWithCovarianceStamped
      >(this->sensorTopic_, 10);

  // Also prepare pose message
  this->poseMsg.header.frame_id = this->linkName_;
  for (int i = 0; i < 36; i++)
    this->poseMsg.pose.covariance[i] = 0;

  this->poseMsg.pose.covariance[0] = this->poseMsg.pose.covariance[7] =
      this->poseMsg.pose.covariance[14] = variance;

  // orientation not available
  this->poseMsg.pose.covariance[21] = this->poseMsg.pose.covariance[28] =
      this->poseMsg.pose.covariance[35] = -1;

  this->pubPose = this->rosNode->advertise<
      geometry_msgs::PoseWithCovarianceStamped
      >(this->sensorTopic_ + "_pose", 10);

  bool isSensorOn = true;
  if (_sdf->HasElement("is_on"))
    isSensorOn = _sdf->GetElement("is_on")->Get<bool>();

  this->InitSwitchablePlugin(this->sensorTopic_, isSensorOn);
}

bool GazeboRptRosPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  bool measurementOK = GazeboRPTPlugin::OnUpdate(_info);

  // Publish sensor state
  this->PublishState();

  if (!measurementOK || !this->IsOn())
    return false;

  // publish position message
  this->posMsg.header.stamp.sec = _info.simTime.sec;
  this->posMsg.header.stamp.nsec = _info.simTime.nsec;

  this->posMsg.pos.pos.x = this->position.x;
  this->posMsg.pos.pos.y = this->position.y;
  this->posMsg.pos.pos.z = this->position.z;

  this->pubPos.publish(this->posMsg);

  // publish pose message
  this->poseMsg.header = this->posMsg.header;

  this->poseMsg.pose.pose.position.x = this->position.x;
  this->poseMsg.pose.pose.position.y = this->position.y;
  this->poseMsg.pose.pose.position.z = this->position.z;

  this->pubPose.publish(this->poseMsg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRptRosPlugin);
}
