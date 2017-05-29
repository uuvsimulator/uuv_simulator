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

/// \file HydrodynamicModel.hh
/// \brief This file contains the definition for different classes of
/// hydrodynamic models for submerged objects

#ifndef __UUV_GAZEBO_HYDRO_MODEL_HH__
#define __UUV_GAZEBO_HYDRO_MODEL_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Shape.hh>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <string>
#include <vector>
#include <map>

#include <uuv_gazebo_plugins/Def.hh>
#include <uuv_gazebo_plugins/BuoyantObject.hh>


namespace gazebo
{
class HydrodynamicModel : public BuoyantObject
{
  /// \brief Protected constructor: Use the factory for object creation.
  protected: HydrodynamicModel(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Returns type of model
  public: virtual std::string GetType() = 0;

  /// \brief Computation of the hydrodynamic forces
  public: virtual void ApplyHydrodynamicForces(
    double time, const math::Vector3 &_flowVelWorld) = 0;

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
    std::string _message = std::string()) = 0;

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
    std::vector<double>& _output) = 0;

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
    double& _output) = 0;

  /// \brief Filter acceleration (fix due to the update structure of Gazebo)
  protected: void ComputeAcc(Eigen::Vector6d _velRel,
                            double _time,
                            double _alpha = 0.3);

  /// \brief Returns true if all parameters are available from the SDF element
  protected: bool CheckParams(sdf::ElementPtr _sdf);

  /// \brief Convert vector to comply with the NED reference frame
  protected: math::Vector3 ToNEDConvention(math::Vector3 _vec);

  /// \brief Convert vector to comply with the NED reference frame
  protected: math::Vector3 FromNEDConvention(math::Vector3 _vec);

  /// \brief Filtered linear & angular acceleration vector in link frame.
  /// This is used to prevent the model to become unstable given that Gazebo
  /// only calls the update function at the beginning or at the end of a
  /// iteration of the physics engine
  protected: Eigen::Vector6d filteredAcc;

  /// \brief Last timestamp (in seconds) at which ApplyHydrodynamicForces was
  /// called
  protected: double lastTime;

  /// \brief Last body-fixed relative velocity (nu_R in Fossen's equations)
  protected: Eigen::Vector6d lastVelRel;

  /// \brief List of parameters needed from the SDF element
  protected: std::vector<std::string> params;

  /// \brief Reynolds number (not used by all models)
  protected: double Re;

  /// \brief Temperature (not used by all models)
  protected: double temperature;
};

/// \brief Pointer to model
typedef boost::shared_ptr<HydrodynamicModel> HydrodynamicModelPtr;

/// \brief Function pointer to create a certain a model
typedef HydrodynamicModel* (*HydrodynamicModelCreator)(sdf::ElementPtr, \
                                                       physics::LinkPtr);

/// \brief Factory singleton class that creates a HydrodynamicModel from sdf.
class HydrodynamicModelFactory
{
  /// \brief Create HydrodynamicModel object according to its sdf Description.
  public: HydrodynamicModel* CreateHydrodynamicModel(sdf::ElementPtr _sdf,
                                                     physics::LinkPtr _link);

  /// \brief Returns the singleton instance of this factory.
  public: static HydrodynamicModelFactory& GetInstance();

  /// \brief Register a class with its creator.
  public: bool RegisterCreator(const std::string& _identifier,
                               HydrodynamicModelCreator _creator);

  /// \brief Constructor is private since this is a singleton.
  private: HydrodynamicModelFactory() {}

  /// \brief Map of each registered identifier to its corresponding creator.
  private: std::map<std::string, HydrodynamicModelCreator> creators_;
};

/// Use the following macro within a HydrodynamicModel declaration:
#define REGISTER_HYDRODYNAMICMODEL(type) static const bool registeredWithFactory

/// Use the following macro before a HydrodynamicModel's definition:
#define REGISTER_HYDRODYNAMICMODEL_CREATOR(type, creator) \
  const bool type::registeredWithFactory = \
  HydrodynamicModelFactory::GetInstance().RegisterCreator( \
  type::IDENTIFIER, creator);

//////////////////////////////////////////////////////////////////////////////
/// \brief Class containting the methods and attributes
/// for a Fossen robot-like hydrodynamic model. The restoring
/// forces are applied by the BuoyantObject class methods. Using the
/// plugin for UUV models will use both this and the buoyant object
/// class definitions, therefore the restoring forces were not
/// inherited here.
/// References:
///     - Fossen, Thor, "Handbook of Marine Craft and Hydrodynamics and Motion
///       Control", 2011
class HMFossen : public HydrodynamicModel
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static HydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
                                std::vector<double>& _output);

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag, double& _output);

  /// \brief Register this model with the factory.
  protected: REGISTER_HYDRODYNAMICMODEL(HMFossen);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  protected: HMFossen(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Computation of the hydrodynamic forces
  public: virtual void ApplyHydrodynamicForces(double time,
                            const math::Vector3 &_flowVelWorld);

  /// \brief Computes the added-mass Coriolis matrix Ca.
  protected: void ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel,
                                             const Eigen::Matrix6d& _Ma,
                                             Eigen::Matrix6d &_Ca) const;

  /// \brief Updates the damping matrix for the current velocity
  protected: void ComputeDampingMatrix(const Eigen::Vector6d& _vel,
                                       Eigen::Matrix6d &_D) const;

  /// \brief Added-mass matrix
  protected: Eigen::Matrix6d Ma;

  /// \brief Added-mass associated Coriolis matrix
  protected: Eigen::Matrix6d Ca;

  /// \brief Damping matrix
  protected: Eigen::Matrix6d D;

  /// \brief Linear damping matrix
  protected: Eigen::Matrix6d DLin;

  /// \brief Linear damping matrix proportional only to the forward speed
  /// (useful for modeling AUVs). From [1], according to Newman (1977), there
  /// is a damping force component that linearly increases with the presence
  /// of forward speed, particularly so for slender bodies.
  ///
  /// References:
  /// [1] Refsnes - 2007 - Nonlinear model-based control of slender body AUVs
  protected: Eigen::Matrix6d DLinForwardSpeed;

  /// \brief Nonlinear damping coefficients
  protected: Eigen::Matrix6d DNonLin;

  /// \brief Linear damping coefficients
  protected: std::vector<double> linearDampCoef;

  /// \brief Quadratic damping coefficients
  protected: std::vector<double> quadDampCoef;
};

//////////////////////////////////////////////////////////////////////////////
/// \brief Class containing the methods and attributes for a hydrodynamic model
/// for a sphere in the fluid
class HMSphere : public HMFossen
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static HydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Register this model with the factory.
  protected: REGISTER_HYDRODYNAMICMODEL(HMSphere);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  protected: HMSphere(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Sphere radius
  protected: double radius;

  /// \brief Drag coefficient
  protected: double Cd;

  /// \brief Area of the cross section
  protected: double areaSection;
};

//////////////////////////////////////////////////////////////////////////////
/// \brief Class containing the methods and attributes for a hydrodynamic model
/// for a cylinder in the fluid
class HMCylinder : public HMFossen
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static HydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Register this model with the factory.
  private: REGISTER_HYDRODYNAMICMODEL(HMCylinder);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  protected: HMCylinder(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Length of the cylinder
  protected: double length;

  /// \brief Sphere radius
  protected: double radius;

  /// \brief Name of the unit rotation axis (just a tag for x, y or z)
  protected: std::string axis;

  /// \brief Ratio between length and diameter
  protected: double dimRatio;

  /// \brief Approximated drag coefficient for the circular area
  protected: double cdCirc;

  /// \brief Approximated drag coefficient for the rectangular section
  protected: double cdLength;
};

//////////////////////////////////////////////////////////////////////////////
/// \brief Class containing the methods and attributes for a hydrodynamic model
/// for a spheroid in the fluid
/// Reference: Antonelli - Underwater Robots
class HMSpheroid : public HMFossen
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static HydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Register this model with the factory.
  private: REGISTER_HYDRODYNAMICMODEL(HMSpheroid);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  protected: HMSpheroid(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Length of the sphroid
  protected: double length;

  /// \brief Prolate spheroid's smaller radius
  protected: double radius;
};

//////////////////////////////////////////////////////////////////////////////
/// \brief Class containing the methods and attributes for a hydrodynamic model
/// for a box in the fluid
class HMBox : public HMFossen
{
  /// \brief Create model of this type with parameter values from sdf.
  public: static HydrodynamicModel* create(sdf::ElementPtr _sdf,
      physics::LinkPtr _link);

  /// \brief Return (derived) type of hydrodynamic model
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Prints parameters
  public: virtual void Print(std::string _paramName,
                             std::string _message = std::string());

  /// \brief Register this model with the factory.
  private: REGISTER_HYDRODYNAMICMODEL(HMBox);

  /// \brief Unique identifier for this geometry
  protected: static const std::string IDENTIFIER;

  /// \brief Constructor
  protected: HMBox(sdf::ElementPtr _sdf, physics::LinkPtr _link);

  /// \brief Drag coefficient
  protected: double Cd;

  /// \brief Length of the box
  protected: double length;

  /// \brief Width of the box
  protected: double width;

  /// \brief Height of the box
  protected: double height;
};
}

#endif  // __UUV_GAZEBO_HYDRO_MODEL_HH__
