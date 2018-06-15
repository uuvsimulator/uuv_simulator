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

#include <uuv_gazebo_plugins/UmbilicalPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
UmbilicalPlugin::UmbilicalPlugin() : ModelPlugin()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

/////////////////////////////////////////////////
UmbilicalPlugin::~UmbilicalPlugin()
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
void UmbilicalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = _model->GetWorld();

  GZ_ASSERT(_sdf->HasElement("umbilical_model"),
            "Could not find umbilical_model.");
  this->umbilical.reset(
        UmbilicalModelFactory::GetInstance().CreateUmbilicalModel(
          _sdf->GetElement("umbilical_model"), _model));

  this->umbilical->Init();

  // Initialize the transport node
  this->node = transport::NodePtr(new transport::Node());
  std::string worldName;
#if GAZEBO_MAJOR_VERSION >= 8
  worldName = this->world->Name();
#else
  worldName = this->world->GetName();
#endif
  this->node->Init(worldName);

  // If fluid topic is available, subscribe to it
  GZ_ASSERT(_sdf->HasElement("flow_velocity_topic"),
            "Umbilical model requires flow velocity topic");
  std::string flowTopic = _sdf->Get<std::string>("flow_velocity_topic");
  GZ_ASSERT(!flowTopic.empty(),
            "Fluid velocity topic tag cannot be empty");
  this->flowSubscriber = this->node->Subscribe(flowTopic,
           &UmbilicalPlugin::UpdateFlowVelocity, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection =
      event::Events::ConnectWorldUpdateBegin(
        boost::bind(&UmbilicalPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void UmbilicalPlugin::UpdateFlowVelocity(ConstVector3dPtr &_msg)
{
  this->flowVelocity.X() = _msg->x();
  this->flowVelocity.Y() = _msg->y();
  this->flowVelocity.Z() = _msg->z();
}

/////////////////////////////////////////////////
void UmbilicalPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  this->umbilical->OnUpdate(_info, this->flowVelocity);
}

GZ_REGISTER_MODEL_PLUGIN(UmbilicalPlugin);
}
