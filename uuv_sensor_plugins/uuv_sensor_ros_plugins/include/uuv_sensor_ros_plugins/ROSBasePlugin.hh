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

#ifndef __ROS_BASE_PLUGIN_HH__
#define __ROS_BASE_PLUGIN_HH__

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <uuv_sensor_ros_plugins/Common.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <uuv_sensor_ros_plugins_msgs/ChangeSensorState.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo/sensors/Noise.hh>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <chrono>
#include <random>
#include <string>
#include <map>


namespace gazebo
{
  class ROSBasePlugin
  {
    /// \brief Class constructor
    public: ROSBasePlugin();

    /// \brief Class destructor
    public: virtual ~ROSBasePlugin();

    /// \brief Initialize base plugin
    public: bool InitBasePlugin(sdf::ElementPtr _sdf);

    /// \brief Update callback from simulation.
    public: virtual bool OnUpdate(const common::UpdateInfo&) = 0;

    /// \brief Add noise normal distribution to the list
    public: bool AddNoiseModel(std::string _name, double _sigma);

    /// \brief Robot namespace
    protected: std::string robotNamespace;

    /// \brief Name of the sensor's output topic
    protected: std::string sensorOutputTopic;

    /// \brief Pointer to the world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to the update event connection.
    protected: event::ConnectionPtr updateConnection;

    /// \brief (Simulation) time when the last sensor measurement was generated.
    protected: common::Time lastMeasurementTime;

    /// \brief Sensor update rate
    protected: double updateRate;

    /// \brief Noise standard deviation
    protected: double noiseSigma;

    /// \brief Noise amplitude
    protected: double noiseAmp;

    /// \brief Flag set to true if the Gazebo sensors messages are supposed
    /// to be published as well (it can avoid unnecessary overhead in case)
    /// the sensor messages needed are only ROS messages
    protected: bool gazeboMsgEnabled;

    /// \brief Pseudo random number generator
    protected: std::default_random_engine rndGen;

    /// \brief Normal distribution describing the noise models
    protected: std::map<std::string, std::normal_distribution<double>>
      noiseModels;

    /// \brief Flag to control the generation of output messages
    protected: std_msgs::Bool isOn;

    /// \brief ROS node handle for communication with ROS
    protected: boost::shared_ptr<ros::NodeHandle> rosNode;

    /// \brief Gazebo's node handle for transporting measurement  messages.
    protected: transport::NodePtr gazeboNode;

    /// \brief Gazebo's publisher for transporting measurement messages.
    protected: ros::Publisher rosSensorOutputPub;

    /// \brief Gazebo's publisher for transporting measurement messages.
    protected: transport::PublisherPtr gazeboSensorOutputPub;

    /// \brief Service server object
    protected: ros::ServiceServer changeSensorSrv;

    /// \brief ROS publisher for the switchable sensor data
    protected: ros::Publisher pluginStatePub;

    /// \brief Pose of the reference frame wrt world frame
    protected: ignition::math::Pose3d referenceFrame;

    /// \brief ROS subscriber for the TF static reference frame
    protected: ros::Subscriber tfStaticSub;

    /// \brief Frame ID of the reference frame
    protected: std::string referenceFrameID;

    /// \brief Flag set to true if reference frame initialized
    protected: bool isReferenceInit;

    /// \brief Reference link
    protected: physics::LinkPtr referenceLink;

    /// \brief Returns true if the plugin is activated
    protected: bool IsOn();

    /// \brief Publish the current state of the plugin
    protected: void PublishState();

    /// \brief Change sensor state (ON/OFF)
    protected: bool ChangeSensorState(
        uuv_sensor_ros_plugins_msgs::ChangeSensorState::Request& _req,
        uuv_sensor_ros_plugins_msgs::ChangeSensorState::Response& _res);

    /// \brief Callback function for the static TF message
    protected: void GetTFMessage(const tf::tfMessage::ConstPtr &_msg);

    /// \brief Returns noise value for a function with zero mean from the
    /// default Gaussian noise model
    protected: double GetGaussianNoise(double _amp);

    /// \brief Returns noise value for a function with zero mean from a
    /// Gaussian noise model according to the model name
    protected: double GetGaussianNoise(std::string _name, double _amp);

    /// \brief Enables generation of simulated measurement if the timeout
    /// since the last update has been reached
    protected: bool EnableMeasurement(const common::UpdateInfo& _info) const;

    /// \brief Updates the pose of the reference frame wrt the world frame
    protected: void UpdateReferenceFramePose();
  };
}

#endif // __ROS_BASE_PLUGIN_HH__
