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

#include <uuv_sensor_plugins_ros/UnderwaterCameraROSPlugin.hh>

#include <string>

namespace gazebo {

UnderwaterCameraROSPlugin::UnderwaterCameraROSPlugin()
{
}

UnderwaterCameraROSPlugin::~UnderwaterCameraROSPlugin()
{
}

void UnderwaterCameraROSPlugin::Load(sensors::SensorPtr _sensor,
                                     sdf::ElementPtr _sdf)
{
    try {
        UnderwaterCameraPlugin::Load(_sensor, _sdf);
    } catch(gazebo::common::Exception &_e)
    {
        gzerr << "Error loading UnderwaterCameraPlugin" << std::endl;
        return;
    }

    if (!ros::isInitialized())
    {
        gzerr << "Not loading UnderwaterCameraROSPlugin since ROS has not "
              << " been properly initialized." << std::endl;
        return;
    }

    // Copying from DepthCameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->depthCamera;

    GazeboRosCameraUtils::Load(_sensor, _sdf);
}


void UnderwaterCameraROSPlugin::OnNewDepthFrame(const float *_image,
                                                unsigned int _width,
                                                unsigned int _height,
                                                unsigned int _depth,
                                                const std::string &_format)
{
    UnderwaterCameraPlugin::OnNewDepthFrame(_image, _width, _height,
                                            _depth, _format);
}

void UnderwaterCameraROSPlugin::OnNewRGBPointCloud(const float *_pcd,
                                                   unsigned int _width,
                                                   unsigned int _height,
                                                   unsigned int _depth,
                                                   const std::string &_format)
{
    UnderwaterCameraPlugin::OnNewRGBPointCloud(_pcd, _width, _height,
                                               _depth, _format);
}

void UnderwaterCameraROSPlugin::OnNewImageFrame(const unsigned char *_image,
                                                unsigned int _width,
                                                unsigned int _height,
                                                unsigned int _depth,
                                                const std::string &_format)
{
    UnderwaterCameraPlugin::OnNewImageFrame(_image, _width, _height,
                                               _depth, _format);

    if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
      return;

#if GAZEBO_MAJOR_VERSION >= 7
    this->sensor_update_time_ = this->parentSensor->LastUpdateTime();
#else
    this->sensor_update_time_ = this->parentSensor->GetLastUpdateTime();
#endif

    if (!this->parentSensor->IsActive())
    {
      if ((*this->image_connect_count_) > 0)
        // do this first so there's chance for sensor to run 1 frame after
        // activate
        this->parentSensor->SetActive(true);
    }
    else
    {
      if ((*this->image_connect_count_) > 0)
        this->PutCameraData(this->lastImage);
        this->PublishCameraInfo();
    }
}

GZ_REGISTER_SENSOR_PLUGIN(UnderwaterCameraROSPlugin);
}
