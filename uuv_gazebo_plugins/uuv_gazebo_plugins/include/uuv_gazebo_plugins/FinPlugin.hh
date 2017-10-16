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

/// \file FinPlugin.hh
/// \brief Model plugin for description of a submarine's fin.

#ifndef __UUV_GAZEBO_PLUGINS_FIN_PLUGIN_HH__
#define __UUV_GAZEBO_PLUGINS_FIN_PLUGIN_HH__

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <uuv_gazebo_plugins/Dynamics.hh>
#include <uuv_gazebo_plugins/LiftDragModel.hh>

#include "Double.pb.h"

namespace gazebo {

/// \brief Definition of a pointer to the floating point message
typedef const boost::shared_ptr<const uuv_gazebo_plugins_msgs::msgs::Double>
ConstDoublePtr;

class FinPlugin : public gazebo::ModelPlugin
{
    /// \brief Constructor
    public: FinPlugin();

    /// \brief Destructor
    public: virtual ~FinPlugin();

    // Documentation inherited.
    public: virtual void Load(gazebo::physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: void OnUpdate(const gazebo::common::UpdateInfo &_info);

    /// \brief Callback for the input topic subscriber
    protected: void UpdateInput(ConstDoublePtr &_msg);

    /// \brief Reads current velocity topic
    protected: void UpdateCurrentVelocity(ConstVector3dPtr &_msg);

    /// \brief Fin dynamic model
    protected: boost::scoped_ptr<Dynamics> dynamics;

    /// \brief Lift&Drag model
    protected: boost::scoped_ptr<LiftDrag> liftdrag;

    /// \brief Update event
    protected: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Gazebo node
    protected: gazebo::transport::NodePtr node;

    /// \brief The fin joint
    protected: gazebo::physics::JointPtr joint;

    /// \brief The fin link
    protected: gazebo::physics::LinkPtr link;

    /// \brief Subscriber to the reference signal topic.
    protected: gazebo::transport::SubscriberPtr commandSubscriber;

    /// \brief Publisher to the output thrust topic
    protected: gazebo::transport::PublisherPtr anglePublisher;

    /// \brief Force component calculated from the lift and drag module
    protected: gazebo::math::Vector3 finForce;

    /// \brief Latest input command.
    protected: double inputCommand;

    /// \brief Fin ID
    protected: int finID;

    /// \brief Topic prefix
    protected: std::string topicPrefix;

    /// \brief Latest fin angle in [rad].
    protected: double angle;

    /// \brief Time stamp of latest thrust force
    protected: gazebo::common::Time angleStamp;

    /// \brief Subcriber to current message
    protected: gazebo::transport::SubscriberPtr currentSubscriber;

    /// \brief Current velocity vector read from topic
    protected: gazebo::math::Vector3 currentVelocity;
};
}

#endif
