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

/// \file UmbilicalPlugin.hh
/// \brief Model plugin for the umbilical (tether) of an ROV.

#ifndef __UUV_GAZEBO_PLUGINS_UMBILICAL_PLUGIN_HH__
#define __UUV_GAZEBO_PLUGINS_UMBILICAL_PLUGIN_HH__

#include <string>

#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/transport/TransportTypes.hh>

#include "UmbilicalModel.hh"

namespace gazebo
{
class UmbilicalSegment
{
  public:
    UmbilicalSegment() { initSdfSegment(); }

    UmbilicalSegment(const std::string& _name,
                     const std::string& _fromLink,
                     const math::Pose& _fromPose,
                     const math::Pose& _toPose,
                     physics::ModelPtr _model);

    void initSdfSegment();

    physics::LinkPtr link;
    physics::LinkPtr linkA;
    physics::JointPtr jointA;
    physics::JointPtr jointB;

    boost::shared_ptr<UmbilicalSegment> prev, next;

    static sdf::SDFPtr sdfSegment;
};

typedef boost::shared_ptr<UmbilicalSegment> UmbilicalSegmentPtr;

class UmbilicalPlugin : public ModelPlugin
{
  /// \brief Destructor.
  public: UmbilicalPlugin();

  /// \brief Constructor.
  public: ~UmbilicalPlugin();

  /// \brief Load plugin and its configuration from sdf
  protected: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Update callback from simulation.
  protected: virtual void OnUpdate(const common::UpdateInfo&);

  /// \brief Reads flow velocity topic
  protected: void UpdateFlowVelocity(ConstVector3dPtr &_msg);

  /// \brief Pointer to the update event connection.
  protected: event::ConnectionPtr updateConnection;

  /// \brief Pointer to the model structure
  protected: gazebo::physics::ModelPtr model;

  /// \brief Gazebo node
  protected: gazebo::transport::NodePtr node;

  /// \brief Subcriber to flow message
  protected: gazebo::transport::SubscriberPtr flowSubscriber;

  /// \brief Flow velocity vector read from topic
  protected: gazebo::math::Vector3 flowVelocity;

  /// \brief Pointer to UmbilicalModel used in this plugin.
  protected: boost::scoped_ptr<UmbilicalModel> umbilical;
};
}

#endif
