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

#ifndef __UUV_GAZEBO_PLUGINS_ACCELERATIONS_TEST_PLUGIN_H__
#define __UUV_GAZEBO_PLUGINS_ACCELERATIONS_TEST_PLUGIN_H__

#include <map>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>

#include <uuv_gazebo_plugins/HydrodynamicModel.hh>
#include <uuv_gazebo_plugins/Def.hh>

namespace gazebo
{
/// \brief Gazebo model plugin class for underwater objects
class AccelerationsTestPlugin : public gazebo::ModelPlugin
{
  /// \brief Constructor
  public: AccelerationsTestPlugin();

  /// \brief Destructor
  public: virtual ~AccelerationsTestPlugin();

  // Documentation inherited.
  public: virtual void Load(gazebo::physics::ModelPtr _model,
                          sdf::ElementPtr _sdf);

  // Documentation inherited.
  public: virtual void Init();

  /// \brief Update the simulation state.
  /// \param[in] _info Information used in the update event.
  public: void Update(const gazebo::common::UpdateInfo &_info);

  /// \brief Connects the update event callback
  protected: virtual void Connect();

  /// \brief Update event
  protected: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Pointer to the world plugin
  protected: gazebo::physics::WorldPtr world;

  /// \brief Pointer to the model structure
  protected: gazebo::physics::ModelPtr model;

  /// \brief Gazebo node
  protected: gazebo::transport::NodePtr node;

  /// \brief Link of test object
  protected: physics::LinkPtr link;

  // ROS things
  private: boost::scoped_ptr<ros::NodeHandle> rosNode;

  protected: ros::Publisher pub_accel_b_gazebo;
  protected: ros::Publisher pub_accel_b_numeric;

  protected: ros::Publisher pub_accel_w_gazebo;
  protected: ros::Publisher pub_accel_w_numeric;

  /// \brief Velocity of link with respect to world frame in previous time step.
  Eigen::Vector6d last_w_v_w_b;

  /// \brief Time stamp of previous time step.
  common::Time lastTime;
};
}

#endif  // __UUV_GAZEBO_PLUGINS_ACCELERATIONS_TEST_PLUGIN_H__
