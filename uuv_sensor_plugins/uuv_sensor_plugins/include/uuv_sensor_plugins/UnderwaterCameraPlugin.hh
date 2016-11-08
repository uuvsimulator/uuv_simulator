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

#ifndef UUV_SENSOR_PLUGINS_GAZEBO_CAMERA_PLUGIN_H_
#define UUV_SENSOR_PLUGINS_GAZEBO_CAMERA_PLUGIN_H_

#include <random>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "gazebo/msgs/msgs.hh"

namespace gazebo {

/// \brief GazeboCameraPlugin simulates an underwater camera.
class UnderwaterCameraPlugin : public DepthCameraPlugin {
  /// \brief Constructor.
  public: UnderwaterCameraPlugin();

  /// \brief Destructor.
  public: virtual ~UnderwaterCameraPlugin();

  /// \brief Load plugin and its configuration from sdf.
  public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

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

  private: virtual void SimulateUnderwater(const cv::Mat& _inputImage,
                                           const cv::Mat& _inputDepth,
                                           cv::Mat& _outputImage);

  /// Temporarily store pointer to previous depth image.
  protected: const float *lastDepth;

  /// Latest simulated image.
  protected: unsigned char* lastImage;

  /// Depth to range lookup table (LUT)
  protected: float* depth2rangeLUT;

  /// Attenuation constants per channel (RGB)
  protected: float attenuation[3];
  /// Background constants per channel (RGB)
  protected: unsigned char background[3];
};
}

#endif  // UUV_SENSOR_PLUGINS_GAZEBO_CAMERA_PLUGIN_H_
