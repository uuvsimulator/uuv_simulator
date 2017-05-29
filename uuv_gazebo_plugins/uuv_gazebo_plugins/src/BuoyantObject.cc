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
#include <gazebo/math/Box.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>

#include "uuv_gazebo_plugins/BuoyantObject.hh"

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

  this->link = _link;
  // Retrieve the bounding box
  // FIXME(mam0box) Gazebo's bounding box method is NOT working

  // TODO(mam0box) Change the way the bounding box is retrieved,
  // it should come from the physics engine but it is still not resolved
  this->boundingBox = link->GetBoundingBox();

  // Estimate volume, can be overwritten later
  this->SetVolume(-1);
  // Estimate CoB, can be overwritten later
  this->EstimateCoB();

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
  this->volume = this->link->GetInertial()->GetMass() / this->fluidDensity;
  gzmsg << this->link->GetName() << " is neutrally buoyant" << std::endl;
}

/////////////////////////////////////////////////
math::Vector3 BuoyantObject::GetBuoyancyForce(const math::Pose &_pose)
{
  double height = this->boundingBox.GetZLength();
  double z = _pose.pos.z;
  double volume = 0.0;

  // TODO Consider the orientation of the object in the calculation of
  //      submerged volume
  if (z + height / 2 > 0 && z < 0)
    volume = this->volume * (std::fabs(z) + height / 2) / height;
  else if (z + height / 2 < 0)
    volume = this->volume;

  math::Vector3 restoring;
  if (!this->neutrallyBuoyant || volume != this->volume)
    restoring = math::Vector3(0, 0, volume * this->fluidDensity * this->g);
  else if (this->neutrallyBuoyant)
    restoring = math::Vector3(
        0, 0, this->link->GetInertial()->GetMass() * this->g);
  // Store the restoring force vector, if needed
  this->StoreVector(RESTORING_FORCE, restoring);
  return restoring;
}

/////////////////////////////////////////////////
void BuoyantObject::ApplyBuoyancyForce()
{
  // Link's pose
  const math::Pose pose = this->link->GetWorldPose();
  // Get the buoyancy force in world coordinates
  math::Vector3 buoyancyWorld = this->GetBuoyancyForce(pose);
  GZ_ASSERT(!std::isnan(buoyancyWorld.GetLength()), "Buoyancy force is nan");
  this->link->AddForceAtRelativePosition(buoyancyWorld, this->GetCoB());
}

/////////////////////////////////////////////////
void BuoyantObject::SetBoundingBox(const math::Box &_bBox)
{
  this->boundingBox = math::Box(_bBox);

  gzmsg << "New bounding box for " << this->link->GetName() << "::"
    << this->boundingBox << std::endl;
}

/////////////////////////////////////////////////
void BuoyantObject::SetVolume(double _volume)
{
  if (_volume > 0)
    this->volume = _volume;
  else
  {
    // If volume is not given, it will be estimated from the collision
    // geometries
    double v = 0.0;
    for (auto collision : link->GetCollisions())
      v += collision->GetShape()->ComputeVolume();

    this->volume = v;
  }
}

/////////////////////////////////////////////////
double BuoyantObject::GetVolume() { return this->volume; }

/////////////////////////////////////////////////
void BuoyantObject::EstimateCoB()
{
  // User did not provide center of buoyancy,
  // compute it from collision volume.
  double volumeSum = 0.0;
  math::Vector3 weightedPosSum = math::Vector3::Zero;
  for (auto collision : this->link->GetCollisions())
  {
    double volume = collision->GetShape()->ComputeVolume();
    volumeSum += volume;
    weightedPosSum += volume * collision->GetWorldPose().pos;
  }

  this->SetCoB(this->link->GetWorldPose().GetInverse().CoordPositionAdd(
      weightedPosSum / volumeSum));
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
void BuoyantObject::SetCoB(const math::Vector3 &_centerOfBuoyancy)
{
  this->centerOfBuoyancy.Set(_centerOfBuoyancy.x, _centerOfBuoyancy.y,
                             _centerOfBuoyancy.z);
}

/////////////////////////////////////////////////
math::Vector3 BuoyantObject::GetCoB() { return this->centerOfBuoyancy; }

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
    this->hydroWrench[_tag] = math::Vector3(0, 0, 0);
}

/////////////////////////////////////////////////
math::Vector3 BuoyantObject::GetStoredVector(std::string _tag)
{
  if (!this->debugFlag)
    return math::Vector3(0, 0, 0);
  if (this->hydroWrench.count(_tag))
    return this->hydroWrench[_tag];
  else
    return math::Vector3(0, 0, 0);
}

/////////////////////////////////////////////////
void BuoyantObject::StoreVector(std::string _tag, math::Vector3 _vec)
{
  if (!this->debugFlag)
    return;
  // Test if field exists
  if (this->hydroWrench.count(_tag))
    this->hydroWrench[_tag] = _vec;
}

/////////////////////////////////////////////////
bool BuoyantObject::IsSubmerged(const math::Pose &_pose)
{
  double height = this->boundingBox.GetZLength();
  double z = _pose.pos.z;

  if (z > (0.75 * height / 2))
    return false;
  else
    return true;
}

/////////////////////////////////////////////////
bool BuoyantObject::IsNeutrallyBuoyant()
{
  return this->neutrallyBuoyant;
}
}
