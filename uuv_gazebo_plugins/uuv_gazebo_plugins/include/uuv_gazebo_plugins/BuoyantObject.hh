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

/// \file BuoyantObject.hh
/// \brief Description of a buoyant object

#ifndef __UUV_GAZEBO_PLUGINS_BUOYANT_OBJECT_HH__
#define __UUV_GAZEBO_PLUGINS_BUOYANT_OBJECT_HH__

#include <string>
#include <map>

#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Shape.hh>

#define RESTORING_FORCE   "restoring_force"

namespace gazebo
{
/// \brief Class describing the dynamics of a buoyant object, useful for simple
/// representations of underwater structures
class BuoyantObject
{
  /// \brief Constructor
  public: BuoyantObject(physics::LinkPtr _link);

  /// \brief Destructor
  public: ~BuoyantObject();

  /// \brief Returns the buoyancy force vector in the world frame
  public: void GetBuoyancyForce(const math::Pose &_pose,
    math::Vector3 &buoyancyForce, math::Vector3 &buoyancyTorque);

  /// \brief Applies buoyancy force on link
  public: void ApplyBuoyancyForce();

  /// \brief Sets the link's submerged volume
  public: void SetVolume(double _volume = -1);

  /// \brief Returns the stored link submerged volume
  public: double GetVolume();

  /// \brief Sets the fluid density in kg/m^3
  public: void SetFluidDensity(double _fluidDensity);

  /// \brief Returns the stored fluid density
  public: double GetFluidDensity();

  /// \brief Sets the position of the center of buoyancy on the body frame
  public: void SetCoB(const gazebo::math::Vector3 &_centerOfBuoyancy);

  /// \brief If the center of buoyancy is not given, it can be estimated from
  /// collision geometries
  public: void EstimateCoB();

  /// \brief Returns the stored center of buoyancy
  public: gazebo::math::Vector3 GetCoB();

  /// \brief Set acceleration of gravity
  public: void SetGravity(double _g);

  /// \brief Get stored acceleration of gravity
  public: double GetGravity();

  /// \brief Sets bounding box
  public: void SetBoundingBox(const gazebo::math::Box &_bBox);

  /// \brief Adds a field in the hydroWrench map
  public: void SetStoreVector(std::string _tag);

  /// \brief Get vector from the hydroWrench map
  public: gazebo::math::Vector3 GetStoredVector(std::string _tag);

  /// \brief Set debug flag to store intermediate forces and torques
  public: void SetDebugFlag(bool _debugOn = true);

  /// \brief Returns true if the robot is completely submerged
  public: bool IsSubmerged();

  /// \brief Returns true if the link was set to be neutrally buoyant
  public: bool IsNeutrallyBuoyant();

  /// \brief Returns the debug flag
  public: bool GetDebugFlag();

  /// \brief Sets this link as neutrally buoyant
  public: void SetNeutrallyBuoyant();

  /// \brief Store vector in the hydroWrench map if the field has been created
  protected: void StoreVector(std::string _tag, gazebo::math::Vector3 _vec);

  /// \brief Volume of fluid displaced by the submerged object
  protected: double volume;

  /// \brief Fluid density
  protected: double fluidDensity;

  /// \brief Acceleration of gravity
  protected: double g;

  /// \brief Center of buoyancy in the body frame
  protected: gazebo::math::Vector3 centerOfBuoyancy;

  /// \brief TEMP for calculation of the buoyancy
  /// force close to the surface
  protected: gazebo::math::Box boundingBox;

  /// \brief Storage for hydrodynamic and hydrostatic forces and torques
  /// for debugging purposes
  protected: std::map<std::string, gazebo::math::Vector3> hydroWrench;

  /// \brief Debug flag, storing all intermediate forces and torques
  protected: bool debugFlag;

  /// \brief Is submerged flag
  protected: bool isSubmerged;

  /// \brief Pointer to the correspondent robot link
  protected: physics::LinkPtr link;

  /// \brief If true, the restoring force will be equal to the gravitational
  // force
  protected: bool neutrallyBuoyant;

  // \brief Metacentric width of the robot, used only for surface vessels and
  // floating objects
  protected: double metacentricWidth;

  /// \brief Metacentric length of the robot, used only for surface vessels and
  /// floating objects
  protected: double metacentricLength;

  /// \brief If the cross section area around water level of the surface vessel
  /// is not given, it will be computed from the object's bounding box
  protected: double waterLevelPlaneArea;

  /// \brief Height of the robot that is submerged (only for surface vessels)
  protected: double submergedHeight;

  /// \brief Flag set to true if the information about the metacentric width and
  /// height is available
  protected: bool isSurfaceVessel;
};
}

#endif
