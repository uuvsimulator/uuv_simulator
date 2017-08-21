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

#include <uuv_sensor_plugins/UnderwaterCameraPlugin.hh>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <string>

#include <boost/bind.hpp>

#include "Common.hh"

namespace gazebo {

UnderwaterCameraPlugin::UnderwaterCameraPlugin()
    : DepthCameraPlugin(),
      lastImage(nullptr)
{
}

UnderwaterCameraPlugin::~UnderwaterCameraPlugin()
{
    if (lastImage)
        delete [] lastImage;

    if (depth2rangeLUT)
        delete [] depth2rangeLUT;
}


void UnderwaterCameraPlugin::Load(sensors::SensorPtr _sensor,
                                  sdf::ElementPtr _sdf)
{
    DepthCameraPlugin::Load(_sensor, _sdf);

    lastImage = new unsigned char[this->width*this->height*this->depth];

    // Only need to load settings specific to this sensor.
    getSdfParam<float>(_sdf, "attenuationR", attenuation[0], 1.f/30.f);
    getSdfParam<float>(_sdf, "attenuationG", attenuation[1], 1.f/30.f);
    getSdfParam<float>(_sdf, "attenuationB", attenuation[2], 1.f/30.f);

    getSdfParam<unsigned char>(_sdf, "backgroundR", background[0], 0);
    getSdfParam<unsigned char>(_sdf, "backgroundG", background[1], 0);
    getSdfParam<unsigned char>(_sdf, "backgroundB", background[2], 0);

    // Compute camera intrinsics fx, fy from FOVs:
#if GAZEBO_MAJOR_VERSION >= 7
    math::Angle hfov = math::Angle(this->depthCamera->HFOV().Radian());
    math::Angle vfov = math::Angle(this->depthCamera->VFOV().Radian());
#else
    math::Angle hfov = this->depthCamera->GetHFOV();
    math::Angle vfov = this->depthCamera->GetVFOV();
#endif

    double fx = (0.5*this->width)/tan(0.5*hfov.Radian());
    double fy = (0.5*this->height)/tan(0.5*vfov.Radian());

    // Assume the camera's principal point to be at the sensor's center:
    double cx = 0.5*this->width;
    double cy = 0.5*this->height;

    // Create and fill depth2range LUT
    this->depth2rangeLUT = new float[this->width*this->height];
    float* lutPtr = this->depth2rangeLUT;
    for (int v = 0; v < this->height; v++)
    {
        double y_z = (v - cy)/fy;
        for (int u = 0; u < this->width; u++)
        {
            double x_z = (u - cx)/fx;
            // Precompute the per-pixel factor in the following formula:
            // range = || (x, y, z) ||_2
            // range = || z * (x/z, y/z, 1.0) ||_2
            // range = z * || (x/z, y/z, 1.0) ||_2
            *(lutPtr++) = sqrt(1.0 + x_z*x_z + y_z*y_z);
        }
    }

    // TODO: Advertise a gazebo topic?
    //    publisher_ = node_handle_->Advertise<sensor_msgs::msgs::Camera>(
    //                sensor_topic_, 10);
}

void UnderwaterCameraPlugin::OnNewDepthFrame(const float *_image,
                                             unsigned int _width,
                                             unsigned int _height,
                                             unsigned int _depth,
                                             const std::string &_format)
{
    // TODO: Can we assume this pointer to always remain valid?
    lastDepth = _image;
}


void UnderwaterCameraPlugin::OnNewRGBPointCloud(const float *_pcd,
                                                unsigned int _width,
                                                unsigned int _height,
                                                unsigned int _depth,
                                                const std::string &_format)
{
}

void UnderwaterCameraPlugin::OnNewImageFrame(const unsigned char *_image,
                                             unsigned int _width,
                                             unsigned int _height,
                                             unsigned int _depth,
                                             const std::string &_format)
{
    // Only create cv::Mat wrappers around existing memory
    // (neither allocates nor copies any images).
    const cv::Mat input(_height, _width, CV_8UC3,
                        const_cast<unsigned char*>(_image));
    const cv::Mat depth(_height, _width, CV_32FC1,
                        const_cast<float*>(lastDepth));

    cv::Mat output(_height, _width, CV_8UC3, lastImage);

    SimulateUnderwater(input, depth, output);

    // TODO: Publish Gazebo topic
}

void UnderwaterCameraPlugin::SimulateUnderwater(const cv::Mat& _inputImage,
                                                const cv::Mat& _inputDepth,
                                                cv::Mat& _outputImage)
{
    const float* lutPtr = this->depth2rangeLUT;
    for (unsigned int row = 0; row < this->height; row++)
    {
        const cv::Vec3b* inrow = _inputImage.ptr<cv::Vec3b>(row);
        const float* depthrow = _inputDepth.ptr<float>(row);
        cv::Vec3b* outrow = _outputImage.ptr<cv::Vec3b>(row);

        for (int col = 0; col < this->width; col++)
        {
            // Convert depth to range using the depth2range LUT
            float r = *(lutPtr++)*depthrow[col];
            const cv::Vec3b& in = inrow[col];
            cv::Vec3b& out = outrow[col];

            if (r < 1e-3)
            {
              r = 1e10;
            }

            for (int c = 0; c < 3; c++)
            {
                // Simplifying assumption: intensity ~ irradiance.
                // This is not really the case but a good enough approximation
                // for now (it would be better to use a proper Radiometric
                // Response Function).
                float e = std::exp(-r*attenuation[c]);
                out[c] = e*in[c] + (1.0f-e)*background[c];
            }
        }
    }
}
GZ_REGISTER_SENSOR_PLUGIN(UnderwaterCameraPlugin);
}
