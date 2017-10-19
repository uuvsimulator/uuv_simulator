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

#ifndef UUV_SENSOR_PLUGINS_ROS_CAMERA_H_
#define UUV_SENSOR_PLUGINS_ROS_CAMERA_H_

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <uuv_sensor_plugins/UnderwaterCameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo {

/// \brief UnderwaterCameraROSPlugin is a ROS wrapper for UnderwaterCameraPlugin
class UnderwaterCameraROSPlugin
        : public UnderwaterCameraPlugin, GazeboRosCameraUtils
{
    /// \brief Constructor.
    public: UnderwaterCameraROSPlugin();

    /// \brief Destructor.
    public: virtual ~UnderwaterCameraROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: virtual void Load(sensors::SensorPtr _sensor,
                              sdf::ElementPtr _sdf);

    public: virtual void OnNewDepthFrame(const float *_image,
          unsigned int _width, unsigned int _height,
          unsigned int _depth, const std::string &_format);

    /// \brief Update the controller
    public: virtual void OnNewRGBPointCloud(const float *_pcd,
          unsigned int _width, unsigned int _height,
          unsigned int _depth, const std::string &_format);

    public: virtual void OnNewImageFrame(const unsigned char *_image,
                        unsigned int _width, unsigned int _height,
                        unsigned int _depth, const std::string &_format);

    private: event::ConnectionPtr loadConnection_;
};
}

#endif  // UUV_SENSOR_PLUGINS_ROS_CAMERA_H_
