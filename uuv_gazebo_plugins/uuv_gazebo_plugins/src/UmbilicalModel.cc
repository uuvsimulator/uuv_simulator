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

#include <uuv_gazebo_plugins/UmbilicalModel.hh>

#include <gazebo/common/Assert.hh>

namespace gazebo
{
/////////////////////////////////////////////////
void UmbilicalModel::Init()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

/////////////////////////////////////////////////
UmbilicalModel* UmbilicalModelFactory::CreateUmbilicalModel(
    sdf::ElementPtr _sdf,
    physics::ModelPtr _model)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
  if (!_sdf->HasElement("type"))
  {
    std::cerr << "umbilical_model does not have an type element"
              << std::endl;
    return NULL;
  }

  std::string identifier = _sdf->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create UmbilicalModel with unknown identifier: "
              << identifier << std::endl;
    return NULL;
  }

  return creators_[identifier](_sdf, _model);
}

/////////////////////////////////////////////////
UmbilicalModelFactory& UmbilicalModelFactory::GetInstance()
{
  static UmbilicalModelFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool UmbilicalModelFactory::RegisterCreator(const std::string& _identifier,
                               UmbilicalModelCreator _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering UmbilicalModel with identifier: "
              << _identifier << " twice" << std::endl;
  }
  creators_[_identifier] = _creator;

  std::cout << "Registered UmbilicalModel type "
            << _identifier << std::endl;
  return true;
}

/////////////////////////////////////////////////
const std::string UmbilicalModelBerg::IDENTIFIER = "Berg";
REGISTER_UMBILICALMODEL_CREATOR(UmbilicalModelBerg, &UmbilicalModelBerg::create)

UmbilicalModelBerg::UmbilicalModelBerg(sdf::ElementPtr _sdf,
                                       physics::ModelPtr _model)
{
  GZ_ASSERT(_sdf->HasElement("connector_link"),
            "Could not find connector_link.");
  std::string connectorLinkName = _sdf->Get<std::string>("connector_link");
  this->connector = _model->GetLink(connectorLinkName);
  GZ_ASSERT(this->connector, "connector_link is invalid");

  GZ_ASSERT(_sdf->HasElement("diameter"), "Could not find diameter.");
  this->diameter = _sdf->Get<double>("diameter");

  GZ_ASSERT(_sdf->HasElement("water_density"), "Could not find water_density.");
  this->rho = _sdf->Get<double>("water_density");
}

/////////////////////////////////////////////////
UmbilicalModel* UmbilicalModelBerg::create(sdf::ElementPtr _sdf,
                                           physics::ModelPtr _model)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return new UmbilicalModelBerg(_sdf, _model);
}

/////////////////////////////////////////////////
void UmbilicalModelBerg::OnUpdate(const common::UpdateInfo &_info,
                                  const gazebo::math::Vector3& _flow)
{
  const math::Pose& pose = this->connector->GetWorldPose();

  double h = -pose.pos.z;

  GZ_ASSERT(h < 10.0,  // Allow for some wiggle room when the UUV emerges.
            "z coordinate should be negative");

  const math::Vector3& uvR = _flow - this->connector->GetWorldLinearVel();

  double uR2 = uvR.x*std::abs(uvR.x);
  double vR2 = uvR.y*std::abs(uvR.y);
  double factor = 0.25*1.2*this->rho;

  math::Vector3 fWorld(uR2, vR2, 0.);
  this->connector->AddForce(fWorld*factor);
}
}
