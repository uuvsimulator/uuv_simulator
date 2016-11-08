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

#ifndef UUV_SENSOR_PLUGINS_ROS_SUBSEAPRESSURE_H_
#define UUV_SENSOR_PLUGINS_ROS_SUBSEAPRESSURE_H_

#include <uuv_sensor_plugins/SubseapressurePlugin.hh>

#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

namespace gazebo {

/// \brief GazeboImuRosPlugin is a ROS wrapper for GazeboImuPlugin.
/// All it does is in addition to GazeboImuPlugin is
/// publishing simulated measurement via a ROS topic.
class GazeboSubseaPressureRosPlugin : public GazeboSubseaPressurePlugin {
    /// \brief Constructor.
    public: GazeboSubseaPressureRosPlugin();

    /// \brief Destructor.
    public: virtual ~GazeboSubseaPressureRosPlugin();

    /// \brief Load module and read parameters from SDF.
    public: virtual void Load(gazebo::physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    /// \brief Update callback from simulator.
    public: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief ROS node handle for communication with ROS
    protected: boost::scoped_ptr<ros::NodeHandle> rosNode_;

    /// \brief ROS publisher for Magnetometer data.
    protected: ros::Publisher rosPublisher_;
};
}

#endif  // UUV_SENSOR_PLUGINS_ROS_SUBSEAPRESSURE_H_
