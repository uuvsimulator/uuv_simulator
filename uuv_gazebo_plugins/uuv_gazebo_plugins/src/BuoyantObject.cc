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

#include <cmath>

#include <gazebo/gazebo.hh>
#include <uuv_gazebo_plugins/BuoyantObject.hh>

namespace gazebo {

/////////////////////////////////////////////////
BuoyantObject::BuoyantObject(physics::LinkPtr _link)
{
  GZ_ASSERT(_link != NULL, "Invalid link pointer");

  // Initial volume
  this->volume = 0.0;
  // Fluid density for sea water at 0 degrees Celsius
  this->fluidDensity = 1028.0;
  this->g = 9.81;
  this->centerOfBuoyancy.Set(0, 0, 0);
  this->debugFlag = false;
  this->isSubmerged = true;
  this->metacentricWidth = 0.0;
  this->metacentricLength = 0.0;
  this->waterLevelPlaneArea = 0.0;
  this->submergedHeight = 0.0;
  this->isSurfaceVessel = false;
  this->scalingVolume = 1.0;
  this->offsetVolume = 0.0;
  this->isSurfaceVesselFloating = false;

  this->link = _link;
  // Retrieve the bounding box
  // FIXME(mam0box) Gazebo's bounding box method is NOT working

  // TODO(mam0box) Change the way the bounding box is retrieved,
  // it should come from the physics engine but it is still not resolved
#if GAZEBO_MAJOR_VERSION >= 8
  this->boundingBox = link->BoundingBox();
#else
  math::Box bBox = link->GetBoundingBox();
  this->boundingBox = ignition::math::Box(bBox.min.x, bBox.min.y, bBox.min.z,
      bBox.max.x, bBox.max.y, bBox.max.z);
#endif
  // Set neutrally buoyant flag to false
  this->neutrallyBuoyant = false;
}

/////////////////////////////////////////////////
BuoyantObject::~BuoyantObject() {}

/////////////////////////////////////////////////
void BuoyantObject::SetNeutrallyBuoyant()
{
  this->neutrallyBuoyant = true;
  // Calculate the equivalent volume for the submerged body
  // so that it will be neutrally buoyant
  double mass;
#if GAZEBO_MAJOR_VERSION >= 8
  mass = this->link->GetInertial()->Mass();
#else
  mass = this->link->GetInertial()->GetMass();
#endif
  this->volume = mass / this->fluidDensity;
  gzmsg << this->link->GetName() << " is neutrally buoyant" << std::endl;
}

/////////////////////////////////////////////////
void BuoyantObject::GetBuoyancyForce(const ignition::math::Pose3d &_pose,
  ignition::math::Vector3d &buoyancyForce,
  ignition::math::Vector3d &buoyancyTorque)
{
  double height = this->boundingBox.ZLength();
  double z = _pose.Pos().Z();
  double volume = 0.0;

  buoyancyForce = ignition::math::Vector3d(0, 0, 0);
  buoyancyTorque = ignition::math::Vector3d(0, 0, 0);

  double mass;
#if GAZEBO_MAJOR_VERSION >= 8
  mass = this->link->GetInertial()->Mass();
#else
  mass = this->link->GetInertial()->GetMass();
#endif

  if (!this->isSurfaceVessel)
  {
    if (z + height / 2 > 0 && z < 0)
    {
      this->isSubmerged = false;
      volume = this->GetVolume() * (std::fabs(z) + height / 2) / height;
    }
    else if (z + height / 2 < 0)
    {
      this->isSubmerged = true;
      volume = this->GetVolume();
    }

    if (!this->neutrallyBuoyant || volume != this->volume)
      buoyancyForce = ignition::math::Vector3d(0, 0,
        volume * this->fluidDensity * this->g);
    else if (this->neutrallyBuoyant)
      buoyancyForce = ignition::math::Vector3d(
          0, 0, mass * this->g);
  }
  else
  {
    // Implementation of the linear (small angle) theory for boxed-shaped
    // vessels. Further details can be seen at
    // T. I. Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control," Apr. 2011.
    // Page 65
    if (this->waterLevelPlaneArea <= 0)
    {
      this->waterLevelPlaneArea = this->boundingBox.XLength() *
        this->boundingBox.YLength();
      gzmsg << this->link->GetName() << "::" << "waterLevelPlaneArea = " <<
        this->waterLevelPlaneArea << std::endl;
    }

    this->waterLevelPlaneArea = mass / (this->fluidDensity * this->submergedHeight);
    double curSubmergedHeight;
    GZ_ASSERT(this->waterLevelPlaneArea > 0.0,
      "Water level plane area must be greater than zero");

    if (z > height / 2.0) {
      // Vessel is completely out of the water
      buoyancyForce = ignition::math::Vector3d(0, 0, 0);
      buoyancyTorque = ignition::math::Vector3d(0, 0, 0);
      return;
    } else if (z < -height / 2.0) {
      curSubmergedHeight = this->boundingBox.ZLength();
    } else {
      curSubmergedHeight = height / 2.0 - z;
    }//else

    volume = curSubmergedHeight * this->waterLevelPlaneArea;
    buoyancyForce = ignition::math::Vector3d(0, 0, volume * this->fluidDensity * this->g);
    buoyancyTorque = ignition::math::Vector3d(
      -1 * this->metacentricWidth * sin(_pose.Rot().Roll()) * buoyancyForce.Z(),
      -1 * this->metacentricLength * sin(_pose.Rot().Pitch()) * buoyancyForce.Z(),
      0);

    // Store the restoring force vector, if needed
    this->StoreVector(RESTORING_FORCE, buoyancyForce);
  }

  // Store the restoring force vector, if needed
  this->StoreVector(RESTORING_FORCE, buoyancyForce);
}

/////////////////////////////////////////////////
void BuoyantObject::ApplyBuoyancyForce()
{
  // Link's pose
  ignition::math::Pose3d pose;
#if GAZEBO_MAJOR_VERSION >= 8
  pose = this->link->WorldPose();
#else
  pose = this->link->GetWorldPose().Ign();
#endif
  // Get the buoyancy force in world coordinates
  ignition::math::Vector3d buoyancyForce, buoyancyTorque;

  this->GetBuoyancyForce(pose, buoyancyForce, buoyancyTorque);

  GZ_ASSERT(!std::isnan(buoyancyForce.Length()),
    "Buoyancy force is invalid");
  GZ_ASSERT(!std::isnan(buoyancyTorque.Length()),
    "Buoyancy torque is invalid");
  if (!this->isSurfaceVessel)
#if GAZEBO_MAJOR_VERSION >= 8
    this->link->AddForceAtRelativePosition(buoyancyForce, this->GetCoB());
#else
    this->link->AddForceAtRelativePosition(
      math::Vector3(buoyancyForce.X(), buoyancyForce.Y(), buoyancyForce.Z()),
      math::Vector3(this->GetCoB().X(), this->GetCoB().Y(), this->GetCoB().Z()));
#endif
  else
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->link->AddForce(buoyancyForce);
    this->link->AddRelativeTorque(buoyancyTorque);
#else
    this->link->AddForce(
      math::Vector3(buoyancyForce.X(), buoyancyForce.Y(), buoyancyForce.Z()));
    this->link->AddRelativeTorque(
      math::Vector3(buoyancyTorque.X(), buoyancyTorque.Y(), buoyancyTorque.Z()));
#endif
  }

}

/////////////////////////////////////////////////
void BuoyantObject::SetBoundingBox(const ignition::math::Box &_bBox)
{
  this->boundingBox = ignition::math::Box(_bBox);

  gzmsg << "New bounding box for " << this->link->GetName() << "::"
    << this->boundingBox << std::endl;
}

/////////////////////////////////////////////////
void BuoyantObject::SetVolume(double _volume)
{
  GZ_ASSERT(_volume > 0, "Invalid input volume");
  this->volume = _volume;
}

/////////////////////////////////////////////////
double BuoyantObject::GetVolume()
{
  return std::max(0.0, this->scalingVolume * (this->volume + this->offsetVolume));
}

/////////////////////////////////////////////////
void BuoyantObject::SetFluidDensity(double _fluidDensity)
{
  GZ_ASSERT(_fluidDensity > 0, "Fluid density must be a positive value");
  this->fluidDensity = _fluidDensity;
}

/////////////////////////////////////////////////
double BuoyantObject::GetFluidDensity() { return this->fluidDensity; }

/////////////////////////////////////////////////
void BuoyantObject::SetCoB(const ignition::math::Vector3d &_centerOfBuoyancy)
{
  this->centerOfBuoyancy = ignition::math::Vector3d(
    _centerOfBuoyancy.X(), _centerOfBuoyancy.Y(), _centerOfBuoyancy.Z());
}

/////////////////////////////////////////////////
ignition::math::Vector3d BuoyantObject::GetCoB() { return this->centerOfBuoyancy; }

/////////////////////////////////////////////////
void BuoyantObject::SetGravity(double _g)
{
  GZ_ASSERT(_g > 0, "Acceleration of gravity must be positive");
  this->g = _g;
}

/////////////////////////////////////////////////
double BuoyantObject::GetGravity() { return this->g; }

/////////////////////////////////////////////////
void BuoyantObject::SetDebugFlag(bool _debugOn) { this->debugFlag = _debugOn; }

/////////////////////////////////////////////////
bool BuoyantObject::GetDebugFlag() { return this->debugFlag; }

/////////////////////////////////////////////////
void BuoyantObject::SetStoreVector(std::string _tag)
{
  if (!this->debugFlag)
    return;
  // Test if field exists
  if (!this->hydroWrench.count(_tag))
    this->hydroWrench[_tag] = ignition::math::Vector3d(0, 0, 0);
}

/////////////////////////////////////////////////
ignition::math::Vector3d BuoyantObject::GetStoredVector(std::string _tag)
{
  if (!this->debugFlag)
    return ignition::math::Vector3d(0, 0, 0);
  if (this->hydroWrench.count(_tag))
    return this->hydroWrench[_tag];
  else
    return ignition::math::Vector3d(0, 0, 0);
}

/////////////////////////////////////////////////
void BuoyantObject::StoreVector(std::string _tag,
  ignition::math::Vector3d _vec)
{
  if (!this->debugFlag)
    return;
  // Test if field exists
  if (this->hydroWrench.count(_tag))
    this->hydroWrench[_tag] = _vec;
}

/////////////////////////////////////////////////
bool BuoyantObject::IsSubmerged()
{
  return this->isSubmerged;
}

/////////////////////////////////////////////////
bool BuoyantObject::IsNeutrallyBuoyant()
{
  return this->neutrallyBuoyant;
}
}
