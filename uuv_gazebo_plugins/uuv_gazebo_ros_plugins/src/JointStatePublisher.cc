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

#include <uuv_gazebo_ros_plugins/JointStatePublisher.hh>

namespace uuv_simulator_ros
{
GZ_REGISTER_MODEL_PLUGIN(JointStatePublisher)

JointStatePublisher::JointStatePublisher()
{
    this->model = NULL;
    this->world = NULL;
}

JointStatePublisher::~JointStatePublisher()
{
    this->node->shutdown();
}

void JointStatePublisher::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  this->model = _parent;

  GZ_ASSERT(this->model != NULL, "Invalid model pointer");

  this->world = this->model->GetWorld();

  if (!ros::isInitialized())
  {
    gzerr << "ROS was not initialized. Closing plugin..." << std::endl;
    return;
  }

  this->node = boost::shared_ptr<ros::NodeHandle>(
    new ros::NodeHandle(this->robotNamespace));
  // Retrieve the namespace used to publish the joint states
  if (_sdf->HasElement("robotNamespace"))
    this->robotNamespace = _sdf->Get<std::string>("robotNamespace");
  else
    this->robotNamespace = this->model->GetName();

  gzmsg << "JointStatePublisher::robotNamespace="
    << this->robotNamespace << std::endl;

  if (this->robotNamespace[0] != '/')
    this->robotNamespace = "/" + this->robotNamespace;

  if (_sdf->HasElement("updateRate"))
    this->updateRate = _sdf->Get<double>("updateRate");
  else
    this->updateRate = 50;

  gzmsg << "JointStatePublisher::Retrieving moving joints:" << std::endl;
  this->movingJoints.clear();
  for (auto &joint : this->model->GetJoints())
  {
    if (joint->GetLowerLimit(0).Radian() == 0 && joint->GetUpperLimit(0).Radian() == 0)
      continue;
    else if (joint->GetType() == gazebo::physics::Base::EntityType::FIXED_JOINT)
      continue;
    else
    {
      this->movingJoints.push_back(joint->GetName());
      gzmsg << "\t- " << joint->GetName() << std::endl;
    }
  }

  GZ_ASSERT(this->updateRate > 0, "Update rate must be positive");

  // Setting the update period
  this->updatePeriod = 1.0 / this->updateRate;

  // Advertise the joint states topic
  this->jointStatePub =
    this->node->advertise<sensor_msgs::JointState>(
      this->robotNamespace + "/joint_states", 1);

  this->lastUpdate = this->world->GetSimTime();
  // Connect the update function to the Gazebo callback
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&JointStatePublisher::OnUpdate, this, _1));
}

void JointStatePublisher::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
  if (this->world->GetSimTime() - this->lastUpdate >= this->updatePeriod)
  {
    this->PublishJointStates();
    this->lastUpdate = this->world->GetSimTime();
  }
}

void JointStatePublisher::PublishJointStates()
{
  ros::Time stamp = ros::Time::now();
  sensor_msgs::JointState jointState;

  jointState.header.stamp = stamp;
  // Resize containers
  jointState.name.resize(this->model->GetJointCount());
  jointState.position.resize(this->model->GetJointCount());
  jointState.velocity.resize(this->model->GetJointCount());
  jointState.effort.resize(this->model->GetJointCount());

  int i = 0;
  for (auto &joint : this->model->GetJoints())
  {
    if (!this->IsIgnoredJoint(joint->GetName()))
    {
      jointState.name[i] = joint->GetName();
      jointState.position[i] = joint->GetAngle(0).Radian();
      jointState.velocity[i] = joint->GetVelocity(0);
      jointState.effort[i] = joint->GetForce(0);
    }
      else
    {
      jointState.name[i] = joint->GetName();
      jointState.position[i] = 0.0;
      jointState.velocity[i] = 0.0;
      jointState.effort[i] = 0.0;
    }

    ++i;
  }

  this->jointStatePub.publish(jointState);
}

bool JointStatePublisher::IsIgnoredJoint(std::string _jointName)
{
  if (this->movingJoints.empty()) return true;
  for (auto joint : this->movingJoints)
    if (_jointName.compare(joint) == 0)
      return false;
  return true;
}
}
