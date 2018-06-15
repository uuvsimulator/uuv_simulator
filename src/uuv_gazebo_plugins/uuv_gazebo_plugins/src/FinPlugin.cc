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

#include <uuv_gazebo_plugins/FinPlugin.hh>
#include <uuv_gazebo_plugins/Def.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>


GZ_REGISTER_MODEL_PLUGIN(gazebo::FinPlugin)

namespace gazebo {

/////////////////////////////////////////////////
FinPlugin::FinPlugin() : inputCommand(0), angle(0), finID(-1)
{
}

/////////////////////////////////////////////////
FinPlugin::~FinPlugin()
{
  if (this->updateConnection)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->updateConnection.reset();
#else
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#endif
  }
}

/////////////////////////////////////////////////
void FinPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initializing the transport node
  this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 8
  this->node->Init(_model->GetWorld()->Name());
#else
  this->node->Init(_model->GetWorld()->GetName());
#endif

  // Fin ID
  GZ_ASSERT(_sdf->HasElement("fin_id"), "Could not find fin_id parameter.");
  this->finID = _sdf->Get<int>("fin_id");
  GZ_ASSERT(this->finID >= 0, "Fin ID must be greater or equal than zero");

  // Root string for topics
  std::stringstream strs;
  strs << "/" << _model->GetName() << "/fins/" << this->finID << "/";
  this->topicPrefix = strs.str();

  // Input/output topics
  std::string inputTopic, outputTopic;
  if (_sdf->HasElement("input_topic"))
    std::string inputTopic = _sdf->Get<std::string>("input_topic");
  else
    inputTopic = this->topicPrefix + "input";

  if (_sdf->HasElement("output_topic"))
    outputTopic = _sdf->Get<std::string>("output_topic");
  else
    outputTopic = this->topicPrefix + "output";

  GZ_ASSERT(_sdf->HasElement("link_name"), "Could not find link_name.");
  std::string link_name = _sdf->Get<std::string>("link_name");
  this->link = _model->GetLink(link_name);
  GZ_ASSERT(this->link, "link is invalid");

  GZ_ASSERT(_sdf->HasElement("joint_name"), "Could not find joint_name.");
  std::string joint_name = _sdf->Get<std::string>("joint_name");
  this->joint = _model->GetJoint(joint_name);
  GZ_ASSERT(this->joint, "joint is invalid");

  // Dynamic model
  GZ_ASSERT(_sdf->HasElement("dynamics"), "Could not find dynamics.");
  this->dynamics.reset(DynamicsFactory::GetInstance().CreateDynamics(
                         _sdf->GetElement("dynamics")));

  // Lift and drag model
  GZ_ASSERT(_sdf->HasElement("liftdrag"), "Could not find liftdrag");
  this->liftdrag.reset(
        LiftDragFactory::GetInstance().CreateLiftDrag(
          _sdf->GetElement("liftdrag")));

  // Subscribe to current velocity topic
  GZ_ASSERT(_sdf->HasElement("current_velocity_topic"),
    "Could not find current_velocity_topic.");
  std::string currentVelocityTopic =
    _sdf->Get<std::string>("current_velocity_topic");

  GZ_ASSERT(!currentVelocityTopic.empty(),
            "Fluid velocity topic tag cannot be empty");

  gzmsg << "Subscribing to current velocity topic: " << currentVelocityTopic
        << std::endl;
  this->currentSubscriber = this->node->Subscribe(currentVelocityTopic,
    &FinPlugin::UpdateCurrentVelocity, this);

  // Advertise the output topic
  this->anglePublisher = this->node->Advertise<
      uuv_gazebo_plugins_msgs::msgs::Double>(outputTopic);


  // Subscribe to the input signal topic
  this->commandSubscriber = this->node->Subscribe(inputTopic,
                                                &FinPlugin::UpdateInput,
                                                this);

  // Connect the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&FinPlugin::OnUpdate,
                    this, _1));
}

/////////////////////////////////////////////////
void FinPlugin::Init()
{
}

/////////////////////////////////////////////////
void FinPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  GZ_ASSERT(!std::isnan(this->inputCommand),
            "nan in this->inputCommand");

  double upperLimit, lowerLimit;
#if GAZEBO_MAJOR_VERSION >= 8
  upperLimit = this->joint->UpperLimit(0);
  lowerLimit = this->joint->LowerLimit(0);
#else
  upperLimit = this->joint->GetUpperLimit(0).Radian();
  lowerLimit = this->joint->GetLowerLimit(0).Radian();
#endif
  // Limit the input command using the fin joint limits
  this->inputCommand = std::min(upperLimit, this->inputCommand);
  this->inputCommand = std::max(lowerLimit, this->inputCommand);

  // Update dynamics model:
  this->angle = this->dynamics->update(this->inputCommand,
                                       _info.simTime.Double());

  // Determine velocity in lift/drag plane:
  ignition::math::Pose3d finPose;
  ignition::math::Vector3d linVel;
#if GAZEBO_MAJOR_VERSION >= 8
  finPose = this->link->WorldPose();
  linVel = this->link->WorldLinearVel();
#else
  finPose = this->link->GetWorldPose().Ign();
  linVel = this->link->GetWorldLinearVel().Ign();
#endif  
  
  ignition::math::Vector3d ldNormalI = finPose.Rot().RotateVector(
    ignition::math::Vector3d::UnitZ);
      
  ignition::math::Vector3d velI = linVel - this->currentVelocity;
  ignition::math::Vector3d velInLDPlaneI = ldNormalI.Cross(velI.Cross(ldNormalI));
  ignition::math::Vector3d velInLDPlaneL = finPose.Rot().RotateVectorReverse(velInLDPlaneI);

  // Compute lift and drag forces:
  this->finForce = this->liftdrag->compute(velInLDPlaneL);

  this->link->AddRelativeForce(this->finForce);
  // Apply forces at cg (with torques for position shift).

  // Apply new fin angle. Do this last since this sets link's velocity to zero.
  this->joint->SetPosition(0, this->angle);

  this->angleStamp = _info.simTime;
}

/////////////////////////////////////////////////
void FinPlugin::UpdateInput(ConstDoublePtr &_msg)
{
  this->inputCommand = _msg->value();
}

/////////////////////////////////////////////////
void FinPlugin::UpdateCurrentVelocity(ConstVector3dPtr &_msg)
{
  this->currentVelocity.X() = _msg->x();
  this->currentVelocity.Y() = _msg->y();
  this->currentVelocity.Z() = _msg->z();
}
}
