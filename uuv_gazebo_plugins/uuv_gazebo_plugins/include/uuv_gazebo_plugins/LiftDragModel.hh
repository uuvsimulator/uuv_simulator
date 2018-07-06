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

/// \file LiftDragModel.hh
/// \brief Various Lift&Drag models for Fins.

#ifndef __UUV_GAZEBO_PLUGINS_LIFT_DRAG_MODEL_HH__
#define __UUV_GAZEBO_PLUGINS_LIFT_DRAG_MODEL_HH__

#include <map>
#include <string>

#include <gazebo/gazebo.hh>

#include <sdf/sdf.hh>

namespace gazebo
{
/// \brief Abstract base class for Lift&Drag models.
class LiftDrag
{
  /// \brief Protected constructor: Use the factory for object creation.
  protected: LiftDrag() : prevTime(-10.), state(0.) {}

  /// \brief Check for element. Complain and return 0 if it is missing.
  public: static bool CheckForElement(sdf::ElementPtr _sdf,
                                      const std::string& element);

  /// \brief Destructor.
  public: virtual ~LiftDrag() {}

  /// \brief Return (derived) type of lift&drag model.
  public: virtual std::string GetType() = 0;

  /// \brief Compute the lift and drag force.
  public: virtual ignition::math::Vector3d compute(
    const ignition::math::Vector3d &_velL) = 0;

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
    double& _output) = 0;

  /// \brief Return list of all parameters
  public: virtual std::map<std::string, double> GetListParams() = 0;

  /// \brief Time of last state update.
  protected: double prevTime;

  /// \brief Latest state.
  protected: double state;
};

/// \brief Function pointer to create a certain LiftDrag object.
typedef LiftDrag* (*LiftDragCreator)(sdf::ElementPtr);

/// \brief Factory singleton class that creates a LiftDrag from sdf.
class LiftDragFactory
{
  /// \brief Create LiftDrag object according to its sdf Description.
  public: LiftDrag* CreateLiftDrag(sdf::ElementPtr _sdf);

  /// \brief Returns the singleton instance of this factory.
  public: static LiftDragFactory& GetInstance();

  /// \brief Register a LiftDrag class with its creator.
  public: bool RegisterCreator(const std::string& _identifier,
                               LiftDragCreator _creator);

  /// \brief Constructor is private since this is a singleton.
  private: LiftDragFactory() {}

  /// \brief Map of each registered identifier to its corresponding creator.
  private: std::map<std::string, LiftDragCreator> creators_;
};

/// Use the following macro within a LiftDrag declaration:
#define REGISTER_LIFTDRAG(type) static const bool registeredWithFactory

/// Use the following macro before a LiftDrag's definition:
#define REGISTER_LIFTDRAG_CREATOR(type, creator) \
  const bool type::registeredWithFactory = \
  LiftDragFactory::GetInstance().RegisterCreator( \
  type::IDENTIFIER, creator);


/// \brief Basic quadratic (Hugin) lift&drag model, page 18 from [1].
/// [1] Engelhardtsen, Øystein. "3D AUV Collision Avoidance." (2007).
class LiftDragQuadratic : public LiftDrag
{
  /// \brief Create thruster model of this type with parameter values from sdf.
  public: static LiftDrag* create(sdf::ElementPtr _sdf);

  /// \brief Return (derived) type of dynamic system.
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Compute the lift and drag force.
  public: virtual ignition::math::Vector3d compute(const ignition::math::Vector3d &velL);

  /// \brief Register this model with the factory.
  private: REGISTER_LIFTDRAG(LiftDragQuadratic);

  /// \brief Return paramater in scalar form for the given tag
  public: virtual bool GetParam(std::string _tag, double& _output);

  /// \brief Return list of all parameters
  public: virtual std::map<std::string, double> GetListParams();

  /// \brief Unique identifier for this dynamical model
  private: static const std::string IDENTIFIER;

  /// \brief Lift constant
  protected: double liftConstant;

  /// \brief Drag constant
  protected: double dragConstant;

  /// \brief Constructor.
  private: LiftDragQuadratic(double _liftConstant, double _dragConstant)
    : LiftDrag(), liftConstant(_liftConstant), dragConstant(_dragConstant) {}
};

/// \brief Lift&drag model that models lift/drag coeffs using two lines.
/// This is based on Gazebo's LiftDragPlugin but implemented as a derived
/// LiftDrag model to allow using it in combination with the dynamics of a
/// Fin.
class LiftDragTwoLines: public LiftDrag
{
  /// \brief Create thruster model of this type with parameter values from sdf.
  public: static LiftDrag* create(sdf::ElementPtr _sdf);

  /// \brief Return (derived) type of dynamic system.
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Compute the lift and drag force.
  public: virtual ignition::math::Vector3d compute(const ignition::math::Vector3d &_velL);

  /// \brief Register this model with the factory.
  private: REGISTER_LIFTDRAG(LiftDragTwoLines);

  /// \brief Unique identifier for this dynamical model.
  private: static const std::string IDENTIFIER;

  /// \brief Return paramater in scalar form for the given tag
  public: virtual bool GetParam(std::string _tag, double& _output);

  /// \brief Return list of all parameters
  public: virtual std::map<std::string, double> GetListParams();

  /// \brief Airfoil area.
  protected: double area;

  /// \brief Fluid density.
  protected: double fluidDensity;

  /// \brief Original zero angle of attack location.
  protected: double a0;

  /// \brief Stall angle.
  protected: double alphaStall;

  /// \brief Lift coefficient without stall.
  protected: double cla;

  /// \brief Lift coefficient with stall.
  protected: double claStall;

  /// \brief Drag coefficient without stall.
  protected: double cda;

  /// \brief Drag coefficient with stall.
  protected: double cdaStall;

  /// \brief Constructor.
  private: LiftDragTwoLines(double _area, double _fluidDensity, double _a0,
                            double _alphaStall, double _cla, double _claStall,
                            double _cda, double _cdaStall)
    : LiftDrag(), area(_area), fluidDensity(_fluidDensity),
      a0(_a0), alphaStall(_alphaStall),
      cla(_cla), claStall(_claStall),
      cda(_cda), cdaStall(_cdaStall) {}
};
}

#endif
