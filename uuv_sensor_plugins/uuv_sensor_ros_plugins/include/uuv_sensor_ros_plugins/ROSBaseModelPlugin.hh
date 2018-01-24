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

#ifndef __ROS_BASE_MODEL_PLUGIN_HH__
#define __ROS_BASE_MODEL_PLUGIN_HH__

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <uuv_sensor_ros_plugins/ROSBasePlugin.hh>
#include <functional>
#include <memory>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
  class ROSBaseModelPlugin : public ROSBasePlugin, public ModelPlugin
  {
    /// \brief Class constructor
    public: ROSBaseModelPlugin();

    /// \brief Class destructor
    public: virtual ~ROSBaseModelPlugin();

    /// \brief Load plugin and its configuration from sdf,
    protected: virtual void Load(physics::ModelPtr _model,
      sdf::ElementPtr _sdf);

    /// \brief Update callback from simulation.
    protected: virtual bool OnUpdate(const common::UpdateInfo&);

    /// \brief Pointer to the model.
    protected: physics::ModelPtr model;

    /// \brief Pointer to the link.
    protected: physics::LinkPtr link;

    /// \brief True if a the local NED frame needs to be broadcasted
    protected: bool enableLocalNEDFrame;

    /// \brief TF broadcaster for the local NED frame
    protected: tf::TransformBroadcaster * tfBroadcaster;

    /// \brief Pose of the local NED frame wrt link frame
    protected: ignition::math::Pose3d localNEDFrame;

    /// \brief Local NED TF frame
    protected: tf::StampedTransform tfLocalNEDFrame;

    /// \brief Returns true if the base_link_ned frame exists
    protected: void SendLocalNEDTransform();
  };
}

#endif // __ROS_BASE_MODEL_PLUGIN_HH__
