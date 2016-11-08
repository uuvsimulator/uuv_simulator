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

/// \file Dynamics.hh
/// \brief 1D dynamic models

#ifndef __UUV_GAZEBO_PLUGINS_THRUSTER_DYNAMICS_HH__
#define __UUV_GAZEBO_PLUGINS_THRUSTER_DYNAMICS_HH__

#include <map>
#include <string>

#include <sdf/sdf.hh>

namespace gazebo
{
/// \brief Abstract base class for thruster dynamics.
class Dynamics
{
  /// \brief Protected constructor: Use the factory for object creation.
  protected: Dynamics() { Reset(); }

  /// \brief Destructor.
  public: virtual ~Dynamics() {}

  /// \brief Return (derived) type of thruster dynamics.
  public: virtual std::string GetType() = 0;

  /// \brief Update the dynamic model.
  /// \param[in] _cmd The commanded value.
  /// \param[in] _t Time stamp of command.
  public: virtual double update(double _cmd, double _t) = 0;

  // \brief Reset state.
  public: virtual void Reset();

  /// \brief Time of last state update.
  protected: double prevTime;

  /// \brief Latest state.
  protected: double state;
};

/// \brief Function pointer to create a certain thruster dynamics object.
typedef Dynamics* (*DynamicsCreator)(sdf::ElementPtr);

/// \brief Factory singleton class that creates a ThrusterDynamics from sdf.
class DynamicsFactory
{
  /// \brief Create ThrusterDynamics object according to its sdf Description.
  public: Dynamics* CreateDynamics(sdf::ElementPtr _sdf);

  /// \brief Returns the singleton instance of this factory.
  public: static DynamicsFactory& GetInstance();

  /// \brief Register a ThrusterDynamic class with its creator.
  public: bool RegisterCreator(const std::string& _identifier,
                               DynamicsCreator _creator);

  /// \brief Constructor is private since this is a singleton.
  private: DynamicsFactory() {}

  /// \brief Map of each registered identifier to its corresponding creator.
  private: std::map<std::string, DynamicsCreator> creators_;
};

/// Use the following macro within a ThrusterDynamics declaration:
#define REGISTER_DYNAMICS(type) static const bool registeredWithFactory

/// Use the following macro before a ThrusterDynamics's definition:
#define REGISTER_DYNAMICS_CREATOR(type, creator) \
  const bool type::registeredWithFactory = \
  DynamicsFactory::GetInstance().RegisterCreator( \
  type::IDENTIFIER, creator);


/// \brief Trivial (no dynamics) zero-order dynamic system.
class DynamicsZeroOrder : public Dynamics
{
  /// \brief Create thruster model of this type with parameter values from sdf.
  public: static Dynamics* create(sdf::ElementPtr _sdf);

  /// \brief Return (derived) type of dynamic system.
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Update dynamical model given input value and time.
  public: virtual double update(double _cmd, double _t);

  /// \brief Register this model with the factory.
  private: REGISTER_DYNAMICS(DynamicsZeroOrder);

  /// \brief Unique identifier for this dynamical model
  private: static const std::string IDENTIFIER;

  /// \brief Constructor.
  private: DynamicsZeroOrder() : Dynamics() {}
};


/// \brief First-order dynamic system.
class DynamicsFirstOrder : public Dynamics
{
  /// \brief Create thruster model of this type with parameter values from sdf.
  public: static Dynamics* create(sdf::ElementPtr _sdf);

  /// \brief Return (derived) type of dynamic system.
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Update dynamical model given input value and time.
  public: virtual double update(double _cmd, double _t);

  /// \brief Register this model with the factory.
  private: REGISTER_DYNAMICS(DynamicsFirstOrder);

  /// \brief Unique identifier for this dynamical model
  private: static const std::string IDENTIFIER;

  /// \brief Constructor.
  private: DynamicsFirstOrder(double _tau);

  /// \brief Time constant tau.
  private: double tau;
};


/// \brief Yoerger's dynamic thruster model.
///
/// This is the lumped-parameter model of Yoerger et al.:
/// The influence of thruster dynamics on underwater vehicle behavior and
/// their incorporation into control system design. (1990)
class ThrusterDynamicsYoerger : public Dynamics
{
  /// \brief Create thruster model of this type with parameter values from sdf.
  public: static Dynamics* create(sdf::ElementPtr _sdf);

  /// \brief Return (derived) type of dynamic system.
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Update dynamical model given input value and time.
  public: virtual double update(double _cmd, double _t);

  /// \brief Register this model with the factory.
  private: REGISTER_DYNAMICS(ThrusterDynamicsYoerger);

  /// \brief Unique identifier for this dynamical model.
  private: static const std::string IDENTIFIER;

  /// \brief Constructor.
  private: ThrusterDynamicsYoerger(double alpha, double beta);

  /// \brief Lumped model parameter with no direct physical meaning.
  private: double alpha;

  /// \brief Lumped model parameter with no direct physical meaning.
  private: double beta;
};


/// \brief Bessa's dynamic thruster model.
///
/// This is "Model 2" described in Bessa et al.:
/// Dynamic Positioning of Underwater Robotic Vehicles with Thruster
/// Dynamics Compensation.
class ThrusterDynamicsBessa : public Dynamics
{
  /// \brief Create thruster model of this type with parameter values from sdf.
  public: static Dynamics* create(sdf::ElementPtr _sdf);

  /// \brief Return (derived) type of dynamic system.
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Update dynamical model given input value and time.
  public: virtual double update(double _cmd, double _t);

  /// \brief Register this model with the factory.
  private: REGISTER_DYNAMICS(ThrusterDynamicsBessa);

  /// \brief Unique identifier for this dynamical model.
  private: static const std::string IDENTIFIER;

  /// \brief Constructor.
  private: ThrusterDynamicsBessa(double _Jmsp, double _Kv1, double _kv2,
                                 double _Kt, double _Rm);

  /// \brief Motor-shaft-propeller inertia.
  private: double Jmsp;

  /// \brief Empirically-determined model parameter.
  private: double Kv1;

  /// \brief Empirically-determined model parameter.
  private: double Kv2;

  /// \brief Motor torque constant.
  private: double Kt;

  /// \brief Winding resistance.
  private: double Rm;
};
}

#endif
