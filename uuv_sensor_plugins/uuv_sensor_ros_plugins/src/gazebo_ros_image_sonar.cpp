/*
 * This file was modified from the original version within Gazebo:
 *
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Modifications:
 *
 * Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

//#include <smarc_gazebo_ros_plugins/gazebo_ros_image_sonar.h>
#include <uuv_sensor_ros_plugins/gazebo_ros_image_sonar.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/tf.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosImageSonar)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosImageSonar::GazeboRosImageSonar() : SensorPlugin(), width(0), height(0), depth(0)
{
  this->point_cloud_connect_count_ = 0;
  this->depth_info_connect_count_ = 0;
  this->last_depth_image_camera_info_update_time_ = common::Time(0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosImageSonar::~GazeboRosImageSonar()
{
  this->newDepthFrameConnection.reset();
  this->newRGBPointCloudConnection.reset();
  this->newImageFrameConnection.reset();

  this->parentSensor.reset();
  this->depthCamera.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosImageSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_parent);
  this->depthCamera = this->parentSensor->DepthCamera();

  if (!this->parentSensor)
  {
    gzerr << "DepthCameraPlugin not attached to a depthCamera sensor\n";
    return;
  }

  this->width = this->depthCamera->ImageWidth();
  this->height = this->depthCamera->ImageHeight();
  this->depth = this->depthCamera->ImageDepth();
  this->format = this->depthCamera->ImageFormat();

  this->newDepthFrameConnection = this->depthCamera->ConnectNewDepthFrame(
      std::bind(&GazeboRosImageSonar::OnNewDepthFrame,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  this->newRGBPointCloudConnection = this->depthCamera->ConnectNewRGBPointCloud(
      std::bind(&GazeboRosImageSonar::OnNewRGBPointCloud,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  this->newImageFrameConnection = this->depthCamera->ConnectNewImageFrame(
      std::bind(&GazeboRosImageSonar::OnNewImageFrame,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("depth_camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // copying from DepthCameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->depthCamera;

  // using a different default
  if (!_sdf->HasElement("imageTopicName"))
    this->image_topic_name_ = "ir/image_raw";
  if (!_sdf->HasElement("cameraInfoTopicName"))
    this->camera_info_topic_name_ = "ir/camera_info";

  // point cloud stuff
  if (!_sdf->HasElement("pointCloudTopicName"))
    this->point_cloud_topic_name_ = "points";
  else
    this->point_cloud_topic_name_ = _sdf->GetElement("pointCloudTopicName")->Get<std::string>();

  // depth image stuff
  if (!_sdf->HasElement("depthImageTopicName"))
    this->depth_image_topic_name_ = "depth/image_raw";
  else
    this->depth_image_topic_name_ = _sdf->GetElement("depthImageTopicName")->Get<std::string>();

  if (!_sdf->HasElement("depthImageCameraInfoTopicName"))
    this->depth_image_camera_info_topic_name_ = "depth/camera_info";
  else
    this->depth_image_camera_info_topic_name_ = _sdf->GetElement("depthImageCameraInfoTopicName")->Get<std::string>();

  if (!_sdf->HasElement("pointCloudCutoff"))
    this->point_cloud_cutoff_ = 0.4;
  else
    this->point_cloud_cutoff_ = _sdf->GetElement("pointCloudCutoff")->Get<double>();
  
  if (!_sdf->HasElement("clip")) {
    gzerr << "We do not have clip" << std::endl;
  }
  else {
    gzerr << "We do have clip" << std::endl;
	gzerr << _sdf->GetElement("clip")->GetElement("far")->Get<double>() << std::endl;
  }

  load_connection_ = GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboRosImageSonar::Advertise, this));
  GazeboRosCameraUtils::Load(_parent, _sdf);
}

void GazeboRosImageSonar::Advertise()
{
  ros::AdvertiseOptions point_cloud_ao =
    ros::AdvertiseOptions::create<sensor_msgs::PointCloud2 >(
      this->point_cloud_topic_name_,1,
      boost::bind( &GazeboRosImageSonar::PointCloudConnect,this),
      boost::bind( &GazeboRosImageSonar::PointCloudDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->point_cloud_pub_ = this->rosnode_->advertise(point_cloud_ao);

  ros::AdvertiseOptions depth_image_ao =
    ros::AdvertiseOptions::create< sensor_msgs::Image >(
      this->depth_image_topic_name_,1,
      boost::bind( &GazeboRosImageSonar::DepthImageConnect,this),
      boost::bind( &GazeboRosImageSonar::DepthImageDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_pub_ = this->rosnode_->advertise(depth_image_ao);

  ros::AdvertiseOptions normal_image_ao =
    ros::AdvertiseOptions::create< sensor_msgs::Image >(
      this->depth_image_topic_name_+"_normals",1,
      boost::bind( &GazeboRosImageSonar::NormalImageConnect,this),
      boost::bind( &GazeboRosImageSonar::NormalImageDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->normal_image_pub_ = this->rosnode_->advertise(normal_image_ao);

  ros::AdvertiseOptions multibeam_image_ao =
    ros::AdvertiseOptions::create< sensor_msgs::Image >(
      this->depth_image_topic_name_+"_multibeam",1,
      boost::bind( &GazeboRosImageSonar::MultibeamImageConnect,this),
      boost::bind( &GazeboRosImageSonar::MultibeamImageDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->multibeam_image_pub_ = this->rosnode_->advertise(multibeam_image_ao);

  ros::AdvertiseOptions sonar_image_ao =
    ros::AdvertiseOptions::create< sensor_msgs::Image >(
      this->depth_image_topic_name_+"_sonar",1,
      boost::bind( &GazeboRosImageSonar::SonarImageConnect,this),
      boost::bind( &GazeboRosImageSonar::SonarImageDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->sonar_image_pub_ = this->rosnode_->advertise(sonar_image_ao);

  ros::AdvertiseOptions raw_sonar_image_ao =
    ros::AdvertiseOptions::create< sensor_msgs::Image >(
      this->depth_image_topic_name_+"_raw_sonar",1,
      boost::bind( &GazeboRosImageSonar::RawSonarImageConnect,this),
      boost::bind( &GazeboRosImageSonar::RawSonarImageDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->raw_sonar_image_pub_ = this->rosnode_->advertise(raw_sonar_image_ao);

  ros::AdvertiseOptions depth_image_camera_info_ao =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
        this->depth_image_camera_info_topic_name_,1,
        boost::bind( &GazeboRosImageSonar::DepthInfoConnect,this),
        boost::bind( &GazeboRosImageSonar::DepthInfoDisconnect,this),
        ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_camera_info_pub_ = this->rosnode_->advertise(depth_image_camera_info_ao);
}


////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosImageSonar::PointCloudConnect()
{
  this->point_cloud_connect_count_++;
  (*this->image_connect_count_)++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosImageSonar::PointCloudDisconnect()
{
  this->point_cloud_connect_count_--;
  (*this->image_connect_count_)--;
  if (this->point_cloud_connect_count_ <= 0)
    this->parentSensor->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosImageSonar::DepthImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosImageSonar::DepthImageDisconnect()
{
  this->depth_image_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosImageSonar::NormalImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosImageSonar::NormalImageDisconnect()
{
  this->depth_image_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosImageSonar::MultibeamImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosImageSonar::MultibeamImageDisconnect()
{
  this->depth_image_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosImageSonar::SonarImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosImageSonar::SonarImageDisconnect()
{
  this->depth_image_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosImageSonar::RawSonarImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosImageSonar::RawSonarImageDisconnect()
{
  this->depth_image_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosImageSonar::DepthInfoConnect()
{
  this->depth_info_connect_count_++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosImageSonar::DepthInfoDisconnect()
{
  this->depth_info_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosImageSonar::OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  if (this->parentSensor->IsActive())
  {
    if (this->point_cloud_connect_count_ <= 0 &&
        this->depth_image_connect_count_ <= 0 &&
        (*this->image_connect_count_) <= 0)
    {
      this->parentSensor->SetActive(false);
    }
    else
    {
      if (this->point_cloud_connect_count_ > 0)
        this->FillPointdCloud(_image);

      if (this->depth_image_connect_count_ > 0)
        //this->FillDepthImage(_image);
        this->ComputeSonarImage(_image);
    }
  }
  else
  {
    if (this->point_cloud_connect_count_ > 0 ||
        this->depth_image_connect_count_ <= 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosImageSonar::OnNewRGBPointCloud(const float *_pcd,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  if (!this->parentSensor->IsActive())
  {
    if (this->point_cloud_connect_count_ > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if (this->point_cloud_connect_count_ > 0)
    {
      this->lock_.lock();
      this->point_cloud_msg_.header.frame_id = this->frame_name_;
      this->point_cloud_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
      this->point_cloud_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;
      this->point_cloud_msg_.width = this->width;
      this->point_cloud_msg_.height = this->height;
      this->point_cloud_msg_.row_step = this->point_cloud_msg_.point_step * this->width;

      sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg_);
      pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      pcd_modifier.resize(_width*_height);

      point_cloud_msg_.is_dense = true;

      sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_rgb(point_cloud_msg_, "rgb");

      for (unsigned int i = 0; i < _width; i++)
      {
        for (unsigned int j = 0; j < _height; j++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
        {
          unsigned int index = (j * _width) + i;
          *iter_x = _pcd[4 * index];
          *iter_y = _pcd[4 * index + 1];
          *iter_z = _pcd[4 * index + 2];
          *iter_rgb = _pcd[4 * index + 3];
          if (i == _width /2 && j == _height / 2)
          {
            uint32_t rgb = *reinterpret_cast<int*>(&(*iter_rgb));
            uint8_t r = (rgb >> 16) & 0x0000ff;
            uint8_t g = (rgb >> 8)  & 0x0000ff;
            uint8_t b = (rgb)       & 0x0000ff;
            std::cerr << (int)r << " " << (int)g << " " << (int)b << "\n";
          }
        }
      }

      this->point_cloud_pub_.publish(this->point_cloud_msg_);
      this->lock_.unlock();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosImageSonar::OnNewImageFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  //ROS_ERROR_NAMED("depth_camera", "camera_ new frame %s %s",this->parentSensor_->GetName().c_str(),this->frame_name_.c_str());
  this->sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
      this->PutCameraData(_image);
      // TODO(lucasw) publish camera info with depth image
      // this->PublishCameraInfo(sensor_update_time);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera data to the interface
void GazeboRosImageSonar::FillPointdCloud(const float *_src)
{
  this->lock_.lock();

  this->point_cloud_msg_.header.frame_id = this->frame_name_;
  this->point_cloud_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->point_cloud_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;
  this->point_cloud_msg_.width = this->width;
  this->point_cloud_msg_.height = this->height;
  this->point_cloud_msg_.row_step = this->point_cloud_msg_.point_step * this->width;

  ///copy from depth to point cloud message
  FillPointCloudHelper(this->point_cloud_msg_,
                 this->height,
                 this->width,
                 this->skip_,
                 (void*)_src );

  this->point_cloud_pub_.publish(this->point_cloud_msg_);

  this->lock_.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Put depth image data to the interface
void GazeboRosImageSonar::FillDepthImage(const float *_src)
{
  this->lock_.lock();
  // copy data into image
  this->depth_image_msg_.header.frame_id = this->frame_name_;
  this->depth_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->depth_image_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;

  ///copy from depth to depth image message
  FillDepthImageHelper(this->depth_image_msg_,
                 this->height,
                 this->width,
                 this->skip_,
                 (void*)_src );

  this->depth_image_pub_.publish(this->depth_image_msg_);

  this->lock_.unlock();
}


// Fill depth information
bool GazeboRosImageSonar::FillPointCloudHelper(
    sensor_msgs::PointCloud2 &point_cloud_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(rows_arg*cols_arg);

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg_, "rgb");

  point_cloud_msg.is_dense = true;

  float* toCopyFrom = (float*)data_arg;
  int index = 0;

  double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  double fl = ((double)this->width) / (2.0 *tan(hfov/2.0));

  // convert depth to point cloud
  for (uint32_t j=0; j<rows_arg; j++)
  {
    double pAngle;
    if (rows_arg>1) pAngle = atan2( (double)j - 0.5*(double)(rows_arg-1), fl);
    else            pAngle = 0.0;

    for (uint32_t i=0; i<cols_arg; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
    {
      double yAngle;
      if (cols_arg>1) yAngle = atan2( (double)i - 0.5*(double)(cols_arg-1), fl);
      else            yAngle = 0.0;

      double depth = toCopyFrom[index++];

      // in optical frame
      // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      *iter_x      = depth * tan(yAngle);
      *iter_y      = depth * tan(pAngle);
      if(depth > this->point_cloud_cutoff_)
      {
        *iter_z    = depth;
      }
      else //point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN ();
        point_cloud_msg.is_dense = false;
      }

      // put image color data for each point
      uint8_t*  image_src = (uint8_t*)(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == rows_arg*cols_arg*3)
      {
        // color
        iter_rgb[0] = image_src[i*3+j*cols_arg*3+0];
        iter_rgb[1] = image_src[i*3+j*cols_arg*3+1];
        iter_rgb[2] = image_src[i*3+j*cols_arg*3+2];
      }
      else if (this->image_msg_.data.size() == rows_arg*cols_arg)
      {
        // mono (or bayer?  @todo; fix for bayer)
        iter_rgb[0] = image_src[i+j*cols_arg];
        iter_rgb[1] = image_src[i+j*cols_arg];
        iter_rgb[2] = image_src[i+j*cols_arg];
      }
      else
      {
        // no image
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
    }
  }

  return true;
}

// Fill depth information
bool GazeboRosImageSonar::FillDepthImageHelper(
    sensor_msgs::Image& image_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
  image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg.height = rows_arg;
  image_msg.width = cols_arg;
  image_msg.step = sizeof(float) * cols_arg;
  image_msg.data.resize(rows_arg * cols_arg * sizeof(float));
  image_msg.is_bigendian = 0;

  const float bad_point = std::numeric_limits<float>::quiet_NaN();

  float* dest = (float*)(&(image_msg.data[0]));
  float* toCopyFrom = (float*)data_arg;
  int index = 0;

  // convert depth to point cloud
  for (uint32_t j = 0; j < rows_arg; j++)
  {
    for (uint32_t i = 0; i < cols_arg; i++)
    {
      float depth = toCopyFrom[index++];

      if (depth > this->point_cloud_cutoff_)
      {
        dest[i + j * cols_arg] = depth;
      }
      else //point in the unseeable range
      {
        dest[i + j * cols_arg] = bad_point;
      }
    }
  }
  return true;
}

cv::Mat GazeboRosImageSonar::ComputeNormalImage(cv::Mat& depth)
{
  
  // copy data into image
  this->normal_image_msg_.header.frame_id = this->frame_name_;
  this->normal_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->normal_image_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;

  // filters
  cv::Mat_<float> f1 = (cv::Mat_<float>(3, 3) << 1,  2,  1,
                                                 0,  0,  0,
                                                -1, -2, -1) / 8;

  cv::Mat_<float> f2 = (cv::Mat_<float>(3, 3) << 1, 0, -1,
                                                 2, 0, -2,
                                                 1, 0, -1) / 8;

  cv::Mat f1m, f2m;
  cv::flip(f1, f1m, 0);
  cv::flip(f2, f2m, 1);

  cv::Mat n1, n2;
  cv::filter2D(depth, n1, -1, f1m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(depth, n2, -1, f2m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::Mat no_readings;
  cv::erode(depth == 0, no_readings, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  //cv::dilate(no_readings, no_readings, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  n1.setTo(0, no_readings);
  n2.setTo(0, no_readings);

  std::vector<cv::Mat> images(3);
  cv::Mat white = cv::Mat::ones(depth.rows, depth.cols, CV_32FC1);

  // NOTE: with different focal lengths, the expression becomes
  // (-dzx*fy, -dzy*fx, fx*fy)
  images.at(0) = n1;    //for green channel
  images.at(1) = n2;    //for red channel
  images.at(2) = 1./this->focal_length_*depth; //for blue channel

  cv::Mat normal_image; // = cv::Mat::zeros(depth.rows, depth.cols, CV_32FC3);
  cv::merge(images, normal_image);
 
  // TODO: we should do this on the split images instead
  for (int i = 0; i < normal_image.rows; ++i) {
    for (int j = 0; j < normal_image.cols; ++j) {
      cv::Vec3f& n = normal_image.at<cv::Vec3f>(i, j);
	  n = cv::normalize(n);
   }
  }

  cv::split(normal_image.clone(), images);
  cv::Vec3d minVec, maxVec;
  for (int i = 0; i < 3; ++i) {
    cv::minMaxLoc(images[i], &minVec[i], &maxVec[i]);
	images[i] -= minVec[i];
	images[i] *= 1./(maxVec[i] - minVec[i]);
  }

  cv::merge(images, normal_image);
  cv::Mat normal_image8;
  normal_image.convertTo(normal_image8, CV_8UC3, 255.0);

  cv_bridge::CvImage img_bridge;
  img_bridge = cv_bridge::CvImage(this->normal_image_msg_.header, sensor_msgs::image_encodings::RGB8, normal_image8);
  img_bridge.toImageMsg(this->normal_image_msg_); // from cv_bridge to sensor_msgs::Image

  this->normal_image_pub_.publish(this->normal_image_msg_);

  return normal_image;
}

cv::Mat GazeboRosImageSonar::ConstructSonarImage(cv::Mat& depth, cv::Mat& normals)
{
  std::vector<cv::Mat> images(3);
  cv::split(normals, images);

  float intensity = 100.; // target strength
  float SL = 200.; // source level
  float NL = 30; // noise level
  float DI = 0.0; // directivity index

  if (dist_matrix_.empty()) {
    std::vector<float> t_x, t_y;
    for (int i = 0; i < depth.cols; i++) t_x.push_back((float(i) - this->cx_)/this->focal_length_);
    for (int i = 0; i < depth.rows; i++) t_y.push_back((float(i) - this->cy_)/this->focal_length_);
    cv::Mat X, Y;
    cv::repeat(cv::Mat(t_x).reshape(1,1), t_y.size(), 1, X);
    cv::repeat(cv::Mat(t_y).reshape(1,1).t(), 1, t_x.size(), Y);
    dist_matrix_ = cv::Mat::zeros(depth.rows, depth.cols, CV_32FC1);
    cv::multiply(X, X, X);
    cv::multiply(Y, Y, Y);
    cv::sqrt(X + Y + 1, dist_matrix_);
  }

  // TODO: make these into proper parameters
  cv::Mat TS = intensity*images[2]; // target strength, probably dir should be DI
  cv::Mat TL = 5*depth; // transmission loss
  cv::multiply(TL, dist_matrix_, TL);
  cv::Mat SNR = SL - 2.0*TL - (NL-DI) + TS;
  SNR.setTo(0., SNR < 0.);

  double minVal, maxVal;
  cv::minMaxLoc(SNR, &minVal, &maxVal);
  SNR -= minVal;
  SNR *= 1./(maxVal - minVal);

  cv::Mat sonar_image8;
  SNR.convertTo(sonar_image8, CV_8UC3, 255.0);

  cv_bridge::CvImage img_bridge;
  img_bridge = cv_bridge::CvImage(this->multibeam_image_msg_.header, sensor_msgs::image_encodings::MONO8, sonar_image8);
  img_bridge.toImageMsg(this->multibeam_image_msg_); // from cv_bridge to sensor_msgs::Image

  this->multibeam_image_pub_.publish(this->multibeam_image_msg_);

  return SNR; //sonar_image8;
}

void GazeboRosImageSonar::ApplySpeckleNoise(cv::Mat& scan, float fov)
{
  std::normal_distribution<double> speckle_dist(1.0, 0.1);

  for (int i = 0; i < scan.rows; ++i) {
    for (int j = 0; j < scan.cols; ++j) {
      float& a = scan.at<float>(i, j);
      if (a == 0.) {
		continue;
      }
	  float speckle = fabs(speckle_dist(generator));
      a *= speckle;
	}
  }
}

void GazeboRosImageSonar::ApplySmoothing(cv::Mat& scan, float fov)
{
  int nrolls = 300;
  int window_size = 30;

  if (angle_range_indices_.empty()) {
    angle_range_indices_.resize(scan.rows/1);
    angle_nbr_indices_.resize(scan.rows/2, 0);
    float threshold = tan(fov);
    for (int j = 0; j < scan.cols; ++j) {
      for (int i = 0; i < scan.rows; ++i) {
        float x = fabs(float(scan.cols)/2. - j);
	    float y = scan.rows - i;
        int dist = int(sqrt(x*x+y*y))/2;
        if (dist >= scan.rows/2 || fabs(x)/y > threshold) {
		  continue;
        }
	    angle_range_indices_[dist].push_back(scan.cols*i+j);
	    angle_nbr_indices_[dist] += 1;
      }
    }
  }

  std::discrete_distribution<> range_dist(angle_nbr_indices_.begin(), angle_nbr_indices_.end());
  std::uniform_real_distribution<double> index_dist(0.0, 1.0);

  std::vector<float> kernel(window_size);
  for (int i = 0; i < window_size; ++i) {
    float diff = float(i-window_size/2)/float(window_size/4.);
    kernel[i] = exp(-0.5*diff);
  }

  std::vector<float> conv_results(2*window_size);
  for (int i = 0; i < nrolls; ++i) {
    int sampled_range = range_dist(generator);
	if (angle_nbr_indices_[sampled_range] == 0) {
      continue;
    }
	int sampled_index = int(index_dist(generator)*angle_nbr_indices_[sampled_range]);

	int window_start = std::max(0, sampled_index-window_size);
	int window_end = std::min(angle_nbr_indices_[sampled_range], sampled_index+window_size);
	for (int i = window_start; i < window_end; ++i) {
	  conv_results[i - window_start] = 0.;
	  float conv_mass = 0.;
      for (int j = 0; j < window_size; ++j) {
		int index = i + j - window_size/2;
		if (index >= 0 && index < angle_nbr_indices_[sampled_range]) {
		   conv_results[i - window_start] += kernel[j]*scan.at<float>(angle_range_indices_[sampled_range][index]);
		   conv_mass += kernel[j];
		}
      }
	  if (conv_mass == 0.) {
	    conv_results[i - window_start] = scan.at<float>(angle_range_indices_[sampled_range][i]);
      }
	  else {
	    conv_results[i - window_start] *= 1./conv_mass;
      }
    }

	for (int i = window_start; i < window_end; ++i) {
      scan.at<float>(angle_range_indices_[sampled_range][i]) = conv_results[i - window_start];
    }

  }

}

void GazeboRosImageSonar::ApplyMedianFilter(cv::Mat& scan)
{
  cv::Mat is_zero = scan == 0.;
  cv::Mat is_bg = scan == 0.2;
  cv::bitwise_or(is_zero, is_bg, is_zero);

  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 9));
  cv::Mat scan_dilated;
  cv::dilate(scan, scan_dilated, element, cv::Point(-1, -1), 1, 1, 1);
  //scan.setTo(scan_dilated, is_zero);
  scan_dilated.copyTo(scan, is_zero);
}

cv::Mat GazeboRosImageSonar::ConstructScanImage(cv::Mat& depth, cv::Mat& SNR)
{
  int rows = 400; // TODO: add a parameter for this
  float range = 17.; // TODO: get this from the sensor config instead
  
  float fov = depthCamera->HFOV().Degree();
  int cols = 2*int(float(rows)*sin(M_PI/180.*fov/2.))+20;

  cv::Mat scan = cv::Mat::zeros(rows, cols, CV_32FC1);
  //float fov = 180./M_PI*2.*asin(this->cx_/this->focal_length_);
  cv::Point center(scan.cols/2, scan.rows);
  cv::Size full_axes(scan.rows, scan.rows);
  cv::ellipse(scan, center, full_axes, -90, -fov/2., fov/2., 0.2, -1);
  cv::Size third_axes(scan.rows/3, scan.rows/3);
  cv::ellipse(scan, center, third_axes, -90, -fov/2., fov/2., 0, -1);

  float mapped_range = float(scan.rows);
  
  for (int i = 0; i < depth.rows; ++i) {
    for (int j = 0; j < depth.cols; ++j) {
      float d = depth.at<float>(i, j);
	  //uchar a = SNR.at<uchar>(i, j);
      float a = SNR.at<float>(i, j);
	  if (d == 0 || a == 0) {
		continue;
      }
	  float x = (float(j) - this->cx_)/this->focal_length_;
	  float y = (float(i) - this->cy_)/this->focal_length_;
	  float z = 1.;

	  if (false) {
		z = d;
      }
	  else {
		z = d*sqrt(y*y + z*z);
      }
      x *= z; y *= z;

	  int pi = scan.rows - 1 - int(z/range*mapped_range);
	  int pj = scan.cols/2 + int(x/range*mapped_range);
      if (pi < scan.rows && pi > 0 && pj < scan.cols && pj > 0 && x*x + z*z < range*range) {      
	    scan.at<float>(pi, pj) = a;
      }
    }
  }
  this->ApplyMedianFilter(scan);
  this->ApplySpeckleNoise(scan, fov);
  //this->ApplySmoothing(scan, fov);

  cv_bridge::CvImage img_bridge;
  img_bridge = cv_bridge::CvImage(this->raw_sonar_image_msg_.header, sensor_msgs::image_encodings::TYPE_32FC1, scan);
  img_bridge.toImageMsg(this->raw_sonar_image_msg_); // from cv_bridge to sensor_msgs::Image
  
  this->raw_sonar_image_pub_.publish(this->raw_sonar_image_msg_);

  return scan;
}

cv::Mat GazeboRosImageSonar::ConstructVisualScanImage(cv::Mat& raw_scan)
{
  float fov = depthCamera->HFOV().Degree();
  float mapped_range = float(raw_scan.rows);

  cv::Scalar blue(15, 48, 102);
  cv::Scalar black(0, 0, 0);
  cv::Mat scan(raw_scan.rows, raw_scan.cols, CV_8UC3);
  scan.setTo(blue);
  
  cv::Point center(scan.cols/2, scan.rows);
  cv::Size axes(scan.rows+3, scan.rows+3);
  cv::ellipse(scan, center, axes, -90, -fov/2., fov/2., black, -1); //, int lineType=LINE_8, 0);

  for (int i = 0; i < scan.rows; ++i) {
    for (int j = 0; j < scan.cols; ++j) {
      float a = raw_scan.at<float>(i, j);
      if (a == 0.) {
		continue;
      }

      if (a < 0.8) {
		scan.at<cv::Vec3b>(i, j) = cv::Vec3b(255*1.25*a, 255*0.78*a, 255*0.50*a);
      }
      else if(a < 1.) {
		scan.at<cv::Vec3b>(i, j) = cv::Vec3b(255*a, 255*(1.88*a-0.88), 255*(-1.99*a+1.99));
      }
      else {
		scan.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255*(1.88-0.88), 255*(-1.99+1.99));
      }
   }
  }
  
  cv::Scalar white(255, 255, 255);
  cv::Size axes1(2./3.*scan.rows, 2./3.*scan.rows);
  cv::Size axes2(1./3.*scan.rows, 1./3.*scan.rows);
  cv::ellipse(scan, center, axes, -90, -fov/2.-0.5, fov/2., white, 1, CV_AA); //, int lineType=LINE_8, 0);
  cv::ellipse(scan, center, axes1, -90, -fov/2., fov/2., white, 1, CV_AA); //, int lineType=LINE_8, 0);
  cv::ellipse(scan, center, axes2, -90, -fov/2., fov/2., white, 1, CV_AA); //, int lineType=LINE_8, 0);

  for (int i = 0; i < 5; ++i) {
    float angle = -fov/2.-0.5 + (fov+0.5)*i/float(5-1);
    int cornerx = int(mapped_range*sin(M_PI/180.*angle));
    int cornery = int(mapped_range*cos(M_PI/180.*angle));
    //cv::Point left_corner(scan.cols/2-cornerx, scan.rows-cornery); 
    //cv::Point right_corner(scan.cols/2+cornerx, scan.rows-cornery); 
    cv::Point corner(scan.cols/2+cornerx, scan.rows-cornery); 
    //cv::line(scan, center, left_corner, white, 2);
    //cv::line(scan, center, right_corner, white, 2);
    cv::line(scan, center, corner, white, 1, CV_AA);
  }
  
  cv_bridge::CvImage img_bridge;
  img_bridge = cv_bridge::CvImage(this->sonar_image_msg_.header, sensor_msgs::image_encodings::RGB8, scan);
  img_bridge.toImageMsg(this->sonar_image_msg_); // from cv_bridge to sensor_msgs::Image
  
  this->sonar_image_pub_.publish(this->sonar_image_msg_);

  return scan;
}

////////////////////////////////////////////////////////////////////////////////
// Put depth image data to the interface
void GazeboRosImageSonar::ComputeSonarImage(const float *_src)
{
  this->lock_.lock();
  // copy data into image
  this->depth_image_msg_.header.frame_id = this->frame_name_;
  this->depth_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->depth_image_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;

  // copy from depth to depth image (OpenCV)
  int rows_arg = this->height;
  int cols_arg = this->width;
  int step_arg = this->skip_;

  sensor_msgs::Image image_msg;
  image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg.height = rows_arg;
  image_msg.width = cols_arg;
  image_msg.step = sizeof(float) * cols_arg;
  image_msg.data.resize(rows_arg * cols_arg * sizeof(float));
  image_msg.is_bigendian = 0;

  //cv::Mat depth_image = cv::Mat(rows_arg, cols_arg, CV_32FC1, (float*)_src).clone();
  cv::Mat depth_image(rows_arg, cols_arg, CV_32FC1, (float*)_src);
  
  // publish normal image
  cv::Mat normal_image = this->ComputeNormalImage(depth_image);
  cv::Mat multibeam_image = this->ConstructSonarImage(depth_image, normal_image);
  cv::Mat raw_scan = this->ConstructScanImage(depth_image, multibeam_image);
  cv::Mat visual_scan = this->ConstructVisualScanImage(raw_scan);
  
  cv_bridge::CvImage img_bridge;
  img_bridge = cv_bridge::CvImage(this->depth_image_msg_.header, sensor_msgs::image_encodings::TYPE_32FC1, depth_image);
  img_bridge.toImageMsg(this->depth_image_msg_); // from cv_bridge to sensor_msgs::Image

  this->depth_image_pub_.publish(this->depth_image_msg_);


  this->lock_.unlock();
}

void GazeboRosImageSonar::PublishCameraInfo()
{
  ROS_DEBUG_NAMED("depth_camera", "publishing default camera info, then depth camera info");
  GazeboRosCameraUtils::PublishCameraInfo();

  if (this->depth_info_connect_count_ > 0)
  {
    common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();

    this->sensor_update_time_ = sensor_update_time;
    if (sensor_update_time - this->last_depth_image_camera_info_update_time_ >= this->update_period_)
    {
      this->PublishCameraInfo(this->depth_image_camera_info_pub_);  // , sensor_update_time);
      this->last_depth_image_camera_info_update_time_ = sensor_update_time;
    }
  }
}

//@todo: publish disparity similar to openni_camera_deprecated/src/nodelets/openni_nodelet.cpp.
/*
#include <stereo_msgs/DisparityImage.h>
pub_disparity_ = comm_nh.advertise<stereo_msgs::DisparityImage > ("depth/disparity", 5, subscriberChanged2, subscriberChanged2);
void GazeboRosImageSonar::PublishDisparityImage(const DepthImage& depth, ros::Time time)
{
  stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage > ();
  disp_msg->header.stamp                  = time;
  disp_msg->header.frame_id               = device_->isDepthRegistered () ? rgb_frame_id_ : depth_frame_id_;
  disp_msg->image.header                  = disp_msg->header;
  disp_msg->image.encoding                = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.height                  = depth_height_;
  disp_msg->image.width                   = depth_width_;
  disp_msg->image.step                    = disp_msg->image.width * sizeof (float);
  disp_msg->image.data.resize (disp_msg->image.height * disp_msg->image.step);
  disp_msg->T = depth.getBaseline ();
  disp_msg->f = depth.getFocalLength () * depth_width_ / depth.getWidth ();
  /// @todo Compute these values from DepthGenerator::GetDeviceMaxDepth() and the like
  disp_msg->min_disparity = 0.0;
  disp_msg->max_disparity = disp_msg->T * disp_msg->f / 0.3;
  disp_msg->delta_d = 0.125;
  depth.fillDisparityImage (depth_width_, depth_height_, reinterpret_cast<float*>(&disp_msg->image.data[0]), disp_msg->image.step);
  pub_disparity_.publish (disp_msg);
}
*/


}