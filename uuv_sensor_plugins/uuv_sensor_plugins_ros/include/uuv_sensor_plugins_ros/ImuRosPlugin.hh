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

#ifndef UUV_SENSOR_PLUGINS_ROS_IMU_H_
#define UUV_SENSOR_PLUGINS_ROS_IMU_H_

#include <uuv_sensor_plugins/ImuPlugin.hh>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace gazebo {

/// \brief GazeboImuRosPlugin is a ROS wrapper for GazeboImuPlugin.
/// All it does is in addition to GazeboImuPlugin is
/// publishing simulated measurements via a ROS topic.
class GazeboImuRosPlugin : public GazeboImuPlugin {
    /// \brief Constructor.
    public: GazeboImuRosPlugin();

    /// \brief Destructor.
    public: virtual ~GazeboImuRosPlugin();

    /// \brief Load module and read parameters from SDF.
    public: virtual void Load(gazebo::physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    /// \brief Update callback from simulator.
    public: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief ROS node handle for communication with ROS
    protected: boost::scoped_ptr<ros::NodeHandle> rosNode_;

    /// \brief ROS publisher for IMU data.
    protected: ros::Publisher pubImu_;

    /// \brief Store IMU message since many attributes do not change (cov.).
    protected: sensor_msgs::Imu imuRosMessage_;
};
}

#endif  // UUV_SENSOR_PLUGINS_ROS_IMU_H_
