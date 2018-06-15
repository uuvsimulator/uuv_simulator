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

/// \file UmbilicalModel.hh
/// \brief Various umbilical models.

#ifndef __UUV_GAZEBO_PLUGINS_UMBILICAL_MODEL_HH__
#define __UUV_GAZEBO_PLUGINS_UMBILICAL_MODEL_HH__

#include <string>
#include <map>
#include <gazebo/gazebo.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
class UmbilicalModel
{
  /// \brief Protected constructor: Use the factory instead
  protected: UmbilicalModel() {}

  /// \brief Destructor.
  public: virtual ~UmbilicalModel() {}

  /// \brief Initialize model.
  public: virtual void Init();

  /// \brief Update Umbilical (and apply forces)
  public: virtual void OnUpdate(const common::UpdateInfo &_info,
                                const ignition::math::Vector3d& _flow) = 0;

  /// \brief Gazebo model to which this umbilical belongs.
  protected: physics::ModelPtr model;

  /// \brief Moving connector link of this umbilical.
  protected: physics::LinkPtr connector;
};

/// \brief Function pointer to create a certain conversion function.
typedef UmbilicalModel* (*UmbilicalModelCreator)(sdf::ElementPtr,
                                                 physics::ModelPtr);

/// \brief Factory singleton class that creates an UmbilicalModel from sdf.
class UmbilicalModelFactory
{
  /// \brief Create a ConversionFunction object according to its sdf Description
  public: UmbilicalModel* CreateUmbilicalModel(sdf::ElementPtr _sdf,
                                               physics::ModelPtr _model);

  /// \brief Return the singleton instance of this factory.
  public: static UmbilicalModelFactory& GetInstance();

  /// \brief Register an UmbilicalModel class with its creator.
  public: bool RegisterCreator(const std::string& _identifier,
                               UmbilicalModelCreator _creator);

  /// \brief Constructor is private since this is a singleton.
  private: UmbilicalModelFactory() {}

  /// \brief Map of each registered identifiers to its corresponding creator
  private: std::map<std::string, UmbilicalModelCreator> creators_;
};

/// Use the following macro within a ThrusterDynamics declaration:
#define REGISTER_UMBILICALMODEL(type) \
  static const bool registeredWithFactory

/// Use the following macro before a ThrusterDynamics's definition:
#define REGISTER_UMBILICALMODEL_CREATOR(type, creator) \
  const bool type::registeredWithFactory = \
  UmbilicalModelFactory::GetInstance().RegisterCreator( \
  type::IDENTIFIER, creator);

class UmbilicalModelBerg : public UmbilicalModel
{
  /// \brief Protected constructor: Use the factory instead
  protected: UmbilicalModelBerg(sdf::ElementPtr _sdf,
                                physics::ModelPtr _model);

  /// \brief Create UmbilicalModel according to its description.
  public: static UmbilicalModel* create(sdf::ElementPtr _sdf,
                                        physics::ModelPtr _model);

  /// \brief Update Umbilical (and apply forces)
  public: virtual void OnUpdate(const common::UpdateInfo &_info,
                                const ignition::math::Vector3d& _flow);

  /// \brief Register this UmbilicalModel function with the factory.
  private: REGISTER_UMBILICALMODEL(UmbilicalModelBerg);

  /// \brief The unique identifier of this UmbilicalModel.
  private: static const std::string IDENTIFIER;

  /// \brief Umbilical diameter.
  private: double diameter;

  /// \brief Water density.
  private: double rho;
};
}

#endif  // __UUV_GAZEBO_PLUGINS_UMBILICAL_MODEL_HH__
