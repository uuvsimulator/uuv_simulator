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

/// \file SphericalCoordinatesROSInterfacePlugin.cc

#include <uuv_world_ros_plugins/SphericalCoordinatesROSInterfacePlugin.hh>

namespace gazebo
{

/////////////////////////////////////////////////
SphericalCoordinatesROSInterfacePlugin::SphericalCoordinatesROSInterfacePlugin()
{ }

/////////////////////////////////////////////////
SphericalCoordinatesROSInterfacePlugin::~SphericalCoordinatesROSInterfacePlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->rosPublishConnection);
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void SphericalCoordinatesROSInterfacePlugin::Load(
  physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  GZ_ASSERT(_world != NULL, "World pointer is invalid");
  GZ_ASSERT(_sdf != NULL, "SDF pointer is invalid");

  this->world = _world;
  this->rosNode.reset(new ros::NodeHandle(""));

  // Advertise the service to get origin of the world in spherical coordinates
  this->worldServices["get_origin_spherical_coordinates"] =
    this->rosNode->advertiseService(
      "/gazebo/get_origin_spherical_coordinates",
      &SphericalCoordinatesROSInterfacePlugin::GetOriginSphericalCoord, this);

  // Advertise the service to get origin of the world in spherical coordinates
  this->worldServices["set_origin_spherical_coordinates"] =
    this->rosNode->advertiseService(
      "/gazebo/set_origin_spherical_coordinates",
      &SphericalCoordinatesROSInterfacePlugin::SetOriginSphericalCoord, this);

  this->worldServices["transform_to_spherical_coord"] =
    this->rosNode->advertiseService(
      "/gazebo/transform_to_spherical_coordinates",
      &SphericalCoordinatesROSInterfacePlugin::TransformToSphericalCoord, this);

  this->worldServices["transform_from_spherical_coord"] =
    this->rosNode->advertiseService(
      "/gazebo/transform_from_spherical_coordinates",
      &SphericalCoordinatesROSInterfacePlugin::TransformFromSphericalCoord, this);

  gzmsg << "Spherical coordinates reference=" << std::endl
    << "\t- Latitude [degrees]="
    << this->world->GetSphericalCoordinates()->LatitudeReference().Degree()
    << std::endl
    << "\t- Longitude [degrees]="
    << this->world->GetSphericalCoordinates()->LongitudeReference().Degree()
    << std::endl
    << "\t- Altitude [m]="
    << this->world->GetSphericalCoordinates()->GetElevationReference()
    << std::endl;
}

/////////////////////////////////////////////////
bool SphericalCoordinatesROSInterfacePlugin::TransformToSphericalCoord(
    uuv_world_ros_plugins_msgs::TransformToSphericalCoord::Request& _req,
    uuv_world_ros_plugins_msgs::TransformToSphericalCoord::Response& _res)
{
  ignition::math::Vector3d cartVec = ignition::math::Vector3d(
    _req.input.x, _req.input.y, _req.input.z);

  ignition::math::Vector3d scVec =
    this->world->GetSphericalCoordinates()->SphericalFromLocal(cartVec);

  _res.latitude_deg = scVec.X();
  _res.longitude_deg = scVec.Y();
  _res.altitude = scVec.Z();
  return true;
}

/////////////////////////////////////////////////
bool SphericalCoordinatesROSInterfacePlugin::TransformFromSphericalCoord(
    uuv_world_ros_plugins_msgs::TransformFromSphericalCoord::Request& _req,
    uuv_world_ros_plugins_msgs::TransformFromSphericalCoord::Response& _res)
{
  ignition::math::Vector3d scVec = ignition::math::Vector3d(
    _req.latitude_deg, _req.longitude_deg, _req.altitude);

  ignition::math::Vector3d cartVec =
    this->world->GetSphericalCoordinates()->LocalFromSpherical(scVec);

  _res.output.x = cartVec.X();
  _res.output.y = cartVec.Y();
  _res.output.z = cartVec.Z();
  return true;
}

/////////////////////////////////////////////////
bool SphericalCoordinatesROSInterfacePlugin::GetOriginSphericalCoord(
    uuv_world_ros_plugins_msgs::GetOriginSphericalCoord::Request& _req,
    uuv_world_ros_plugins_msgs::GetOriginSphericalCoord::Response& _res)
{
  _res.latitude_deg =
    this->world->GetSphericalCoordinates()->LatitudeReference().Degree();
  _res.longitude_deg =
    this->world->GetSphericalCoordinates()->LongitudeReference().Degree();
  _res.altitude =
    this->world->GetSphericalCoordinates()->GetElevationReference();
  return true;
}

/////////////////////////////////////////////////
bool SphericalCoordinatesROSInterfacePlugin::SetOriginSphericalCoord(
    uuv_world_ros_plugins_msgs::SetOriginSphericalCoord::Request& _req,
    uuv_world_ros_plugins_msgs::SetOriginSphericalCoord::Response& _res)
{
  ignition::math::Angle angle;
  angle.Degree(_req.latitude_deg);
  this->world->GetSphericalCoordinates()->SetLatitudeReference(angle);

  angle.Degree(_req.longitude_deg);
  this->world->GetSphericalCoordinates()->SetLongitudeReference(angle);

  this->world->GetSphericalCoordinates()->SetElevationReference(_req.altitude);
  _res.success = true;
  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_WORLD_PLUGIN(SphericalCoordinatesROSInterfacePlugin)
}
