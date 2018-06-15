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

#ifndef __THRUSTER_ROS_PLUGIN_HH__
#define __THRUSTER_ROS_PLUGIN_HH__

#include <map>
#include <string>

#include <uuv_gazebo_plugins/ThrusterPlugin.hh>

#include <boost/scoped_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <uuv_gazebo_ros_plugins_msgs/SetThrusterState.h>
#include <uuv_gazebo_ros_plugins_msgs/GetThrusterState.h>
#include <uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiency.h>
#include <uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiency.h>

namespace uuv_simulator_ros
{
  class ThrusterROSPlugin : public gazebo::ThrusterPlugin
  {
    /// \brief Constrcutor.
    public: ThrusterROSPlugin();

    /// \brief Destructor.
    public: ~ThrusterROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Publish thruster state via ROS.
    public: void RosPublishStates();

    /// \brief Set new set point (desired thrust [N]) for thruster.
    public: void SetThrustReference(
        const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr &_msg);

    /// \brief Return the ROS publish period.
    public: gazebo::common::Time  GetRosPublishPeriod();

    /// \brief Set the ROS publish frequency (Hz).
    public: void SetRosPublishRate(double _hz);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Set the thrust efficiency factor
    public: bool SetThrustForceEfficiency(
      uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Request& _req,
      uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Response& _res);

    /// \brief Get the thrust efficiency factor
    public: bool GetThrustForceEfficiency(
      uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency::Request& _req,
      uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency::Response& _res);

    /// \brief Set the dynamic state efficiency factor
    public: bool SetDynamicStateEfficiency(
      uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Request& _req,
      uuv_gazebo_ros_plugins_msgs::SetThrusterEfficiency::Response& _res);

      /// \brief Get the dynamic state efficiency factor
    public: bool GetDynamicStateEfficiency(
        uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency::Request& _req,
        uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency::Response& _res);

    /// \brief Turn thruster on/off
    public: bool SetThrusterState(
      uuv_gazebo_ros_plugins_msgs::SetThrusterState::Request& _req,
      uuv_gazebo_ros_plugins_msgs::SetThrusterState::Response& _res);

    /// \brief Get thruster state
    public: bool GetThrusterState(
      uuv_gazebo_ros_plugins_msgs::GetThrusterState::Request& _req,
      uuv_gazebo_ros_plugins_msgs::GetThrusterState::Response& _res);

    /// \brief Map of thruster services
    private: std::map<std::string, ros::ServiceServer> services;

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief Subscriber reacting to new reference thrust set points.
    private: ros::Subscriber subThrustReference;

    /// \brief Publisher for current actual thrust.
    private: ros::Publisher pubThrust;

    /// \brief Publisher for current actual thrust as wrench.
    private: ros::Publisher pubThrustWrench;

    /// \brief Publisher for the thruster state
    private: ros::Publisher pubThrusterState;

    /// \brief Publisher for the thrust force efficiency
    private: ros::Publisher pubThrustForceEff;

    /// \brief Publisher for the dynamic state efficiency
    private: ros::Publisher pubDynamicStateEff;

    /// \brief Connection for callbacks on update world.
    private: gazebo::event::ConnectionPtr rosPublishConnection;

    /// \brief Period after which we should publish a message via ROS.
    private: gazebo::common::Time rosPublishPeriod;

    /// \brief Last time we published a message via ROS.
    private: gazebo::common::Time lastRosPublishTime;
  };
}

#endif  // __THRUSTER_ROS_PLUGIN_HH__
