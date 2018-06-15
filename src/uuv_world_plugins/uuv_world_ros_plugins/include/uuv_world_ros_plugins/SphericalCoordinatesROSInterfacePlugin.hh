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

/// \file SphericalCoordinatesROSInterfacePlugin.hh

#ifndef __SC_ROS_INTERFACE_PLUGIN_HH__
#define __SC_ROS_INTERFACE_PLUGIN_HH__

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/SphericalCoordinates.hh>
#include <gazebo/physics/World.hh>
#include <ros/ros.h>
#include <uuv_world_ros_plugins_msgs/SetOriginSphericalCoord.h>
#include <uuv_world_ros_plugins_msgs/GetOriginSphericalCoord.h>
#include <uuv_world_ros_plugins_msgs/TransformToSphericalCoord.h>
#include <uuv_world_ros_plugins_msgs/TransformFromSphericalCoord.h>
#include <geometry_msgs/Vector3.h>

#include <map>
#include <string>

namespace gazebo
{

class SphericalCoordinatesROSInterfacePlugin : public WorldPlugin
{
  /// \brief Constructor
  public: SphericalCoordinatesROSInterfacePlugin();

  /// \brief Destructor
  public: virtual ~SphericalCoordinatesROSInterfacePlugin();

  /// \brief Load module and read parameters from SDF.
  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  /// \brief Service call that returns the origin in WGS84 standard
  public: bool GetOriginSphericalCoord(
      uuv_world_ros_plugins_msgs::GetOriginSphericalCoord::Request& _req,
      uuv_world_ros_plugins_msgs::GetOriginSphericalCoord::Response& _res);

  /// \brief Service call that returns the origin in WGS84 standard
  public: bool SetOriginSphericalCoord(
      uuv_world_ros_plugins_msgs::SetOriginSphericalCoord::Request& _req,
      uuv_world_ros_plugins_msgs::SetOriginSphericalCoord::Response& _res);

  /// \brief Service call to transform from Cartesian to spherical coordinates
  public: bool TransformToSphericalCoord(
      uuv_world_ros_plugins_msgs::TransformToSphericalCoord::Request& _req,
      uuv_world_ros_plugins_msgs::TransformToSphericalCoord::Response& _res);

  /// \brief Service call to transform from spherical to Cartesian coordinates
  public: bool TransformFromSphericalCoord(
      uuv_world_ros_plugins_msgs::TransformFromSphericalCoord::Request& _req,
      uuv_world_ros_plugins_msgs::TransformFromSphericalCoord::Response& _res);

  /// \brief Pointer to this ROS node's handle.
  protected: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief Connection for callbacks on update world.
  protected: event::ConnectionPtr rosPublishConnection;

  /// \brief Pointer to world
  protected: physics::WorldPtr world;

  /// \brief All underwater world services
  protected: std::map<std::string, ros::ServiceServer> worldServices;
};

}

#endif // __SC_ROS_INTERFACE_PLUGIN_HH__
