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

#ifndef UUV_SENSOR_PLUGINS_GAZEBO_SENSOR_PLUGIN_H_
#define UUV_SENSOR_PLUGINS_GAZEBO_SENSOR_PLUGIN_H_

#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo {

/// \brief GazeboSensorPlugin is an abstract base class for our sensor plugins.
///
class GazeboSensorPlugin : public ModelPlugin {
  /// \brief Constructor
  public: GazeboSensorPlugin();

  /// \brief Destructor.
  public: virtual ~GazeboSensorPlugin();

  /// \brief Load plugin and its configuration from sdf,
  protected: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Update callback from simulation.
  protected: virtual bool OnUpdate(const common::UpdateInfo&) = 0;

  /// \brief Check whether it is time to generate a simulated measurement.
  protected: virtual bool ShouldIGenerate(
            const common::UpdateInfo& _info) const;

  /// \brief Namespace of this robot.
  protected: std::string namespace_;

  /// \brief The name of the topic to which measurements are published.
  protected: std::string sensorTopic_;

  /// \brief Gazebo's node handle for transporting measurement  messages.
  protected: transport::NodePtr nodeHandle_;

  /// \brief Gazebo's publisher for transporting measurement messages.
  protected: transport::PublisherPtr publisher_;

  /// \brief Frame id i.e. link name of this sensor.
  protected: std::string linkName_;

  protected: std::default_random_engine rndGen_;

  protected: std::normal_distribution<double> normal_;

  /// \brief Pointer to the world.
  protected: physics::WorldPtr world_;

  /// \brief Pointer to the model.
  protected: physics::ModelPtr model_;

  /// \brief Pointer to the link.
  protected: physics::LinkPtr link_;

  /// \brief Pointer to the update event connection.
  protected: event::ConnectionPtr updateConnection_;

  /// \brief (Simulation) time when the last sensor measurement was generated.
  protected: common::Time lastMeasTime_;

  /// \brief Desired time between updates.
  protected: common::Time updatePeriod_;
};
}

#endif  // UUV_SENSOR_PLUGINS_GAZEBO_SENSOR_PLUGIN_H_
