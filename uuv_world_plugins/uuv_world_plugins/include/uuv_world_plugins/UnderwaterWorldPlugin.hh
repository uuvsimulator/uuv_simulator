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

/// \file UnderwaterWorldPlugin.hh
/// \brief Plugin that for the underwater world

#ifndef __UNDERWATER_WORLD_PLUGIN_HH__
#define __UNDERWATER_WORLD_PLUGIN_HH__

#include <map>
#include <cmath>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <uuv_world_plugins/GaussMarkovProcess.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief Class for the underwater world plugin
  class UnderwaterWorldPlugin : public gazebo::WorldPlugin
  {
    /// \brief Class constructor
    public: UnderwaterWorldPlugin();

    /// \brief Class destructor
    public: virtual ~UnderwaterWorldPlugin();

    // Documentation inherited.
    public: virtual void Load(gazebo::physics::WorldPtr _world,
        sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: void Update(const gazebo::common::UpdateInfo &_info);

    /// \brief Publish current velocity and the pose of its frame
    protected: void PublishCurrentVelocity();

    /// \brief Update event
    protected: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Pointer to world
    protected: gazebo::physics::WorldPtr world;

    /// \brief Pointer to sdf
    protected: sdf::ElementPtr sdf;

    /// \brief True if the sea surface is present
    protected: bool hasSurface;

    /// \brief NED (North-East-Down) frame
    protected: gazebo::math::Pose nedFrame;

    /// \brief Pointer to a node for communication
    protected: gazebo::transport::NodePtr node;

    /// \brief Map of publishers
    protected: std::map<std::string, gazebo::transport::PublisherPtr>
      publishers;

    /// \brief NED frame topic
    protected: std::string nedFrameTopic;

    /// \brief Current velocity topic
    protected: std::string currentVelocityTopic;

    /// \brief Namespace for topics and services
    protected: std::string ns;

    /// \brief Gauss-Markov process instance for the current velocity
    protected: GaussMarkovProcess currentVelModel;

    /// \brief Gauss-Markov process instance for horizontal angle model
    protected: GaussMarkovProcess currentHorzAngleModel;

    /// \brief Gauss-Markov process instance for vertical angle model
    protected: GaussMarkovProcess currentVertAngleModel;

    /// \brief Last update time stamp
    protected: gazebo::common::Time lastUpdate;

    /// \brief Current linear velocity vector
    protected: gazebo::math::Vector3 currentVelocity;
  };
}

#endif  // __UNDERWATER_WORLD_PLUGIN_HH__
