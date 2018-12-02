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

#include <uuv_sensor_ros_plugins/ROSBasePlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
ROSBasePlugin::ROSBasePlugin()
{
  this->gazeboMsgEnabled = true;
  this->referenceFrame = ignition::math::Pose3d::Zero;
  this->referenceFrameID = "world";
  this->isReferenceInit = false;
  this->isOn.data = true;
  this->world = NULL;
  this->referenceLink = NULL;

  // Set seed for the noise generator
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  this->rndGen = std::default_random_engine(seed);
}

/////////////////////////////////////////////////
ROSBasePlugin::~ROSBasePlugin()
{
  if (this->rosNode)
    this->rosNode->shutdown();
  if (this->updateConnection)
    this->updateConnection.reset();
}

/////////////////////////////////////////////////
bool ROSBasePlugin::InitBasePlugin(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(this->world != NULL, "World object not available");
  // Get the robot namespace
  GetSDFParam<std::string>(_sdf, "robot_namespace", this->robotNamespace, "");
  GZ_ASSERT(!this->robotNamespace.empty(), "Robot namespace was not provided");

  // Read separately in case a default topic name is given
  std::string sensorTopic;
  GetSDFParam<std::string>(_sdf, "sensor_topic", sensorTopic, "");
  if (!sensorTopic.empty())
    this->sensorOutputTopic = sensorTopic;
  GZ_ASSERT(!this->sensorOutputTopic.empty(),
    "Sensor output topic has not been provided");

  // Get the update rate
  GetSDFParam<double>(_sdf, "update_rate", this->updateRate, 30.0);

  // Get flag to enable generation of Gazebo messages
  GetSDFParam<bool>(_sdf, "enable_gazebo_messages", this->gazeboMsgEnabled,
    true);

  // If output Gazebo messages have been enabled, create a Gazebo node
  this->gazeboNode = transport::NodePtr(new transport::Node());
  this->gazeboNode->Init(this->robotNamespace);

  // Create ROS node
  if (!ros::isInitialized())
  {
    gzerr << "Not loading sensor plugin since ROS has not been properly "
          << "initialized." << std::endl;
    return false;
  }

  this->rosNode.reset(new ros::NodeHandle(this->robotNamespace));

  // Initialize reference frame
  if (_sdf->HasElement("static_reference_frame"))
  {
    GetSDFParam<std::string>(_sdf, "static_reference_frame",
      this->referenceFrameID, "world");
    gzmsg << "Static reference frame=" << this->referenceFrameID << std::endl;
    this->referenceLink = NULL;
    // In case the reference frame provide is different from world, then
    // subscribe to the tf_static topic to acquire the pose of the reference
    // frame.
    if (this->referenceFrameID.compare("world") != 0)
    {
      this->tfStaticSub = this->rosNode->subscribe<tf::tfMessage>(
        "/tf_static", 1,
        boost::bind(&ROSBasePlugin::GetTFMessage, this, _1));
    }
    else
      this->isReferenceInit = true;
  }
  else if (_sdf->HasElement("reference_link_name"))
  {
    GZ_ASSERT(this->referenceLink != NULL,
      "Reference link has not been initialized");
  }
  else
  {
    this->referenceFrameID = "world";
    this->referenceLink = NULL;
    this->isReferenceInit = true;
  }

  // Acquire current simulation time
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastMeasurementTime = this->world->SimTime();
#else
  this->lastMeasurementTime = this->world->GetSimTime();
#endif

  // Initialize the switchable functionality of the sensor
  bool isSensorOn;
  GetSDFParam<bool>(_sdf, "is_on", isSensorOn, true);
  this->isOn.data = isSensorOn;

  // The ROS node is expected to be initialized under a the namespace of the
  // plugin running this module
  this->changeSensorSrv = this->rosNode->advertiseService(
    this->sensorOutputTopic + "/change_state",
    &ROSBasePlugin::ChangeSensorState, this);

  this->pluginStatePub = this->rosNode->advertise<std_msgs::Bool>(
    this->sensorOutputTopic + "/state", 1);

  GetSDFParam<double>(_sdf, "noise_sigma", this->noiseSigma, 0.0);
  GZ_ASSERT(this->noiseSigma >= 0.0,
    "Signal noise sigma must be greater or equal to zero");

  GetSDFParam<double>(_sdf, "noise_amplitude", this->noiseAmp, 0.0);
  GZ_ASSERT(this->noiseAmp >= 0.0,
    "Signal noise amplitude must be greater or equal to zero");

  // Add a default Gaussian noise model
  this->AddNoiseModel("default", this->noiseSigma);
}

/////////////////////////////////////////////////
void ROSBasePlugin::GetTFMessage(const tf::tfMessage::ConstPtr &_msg)
{
  if (this->isReferenceInit)
    return;

  if (_msg->transforms.size() > 0)
  {
    for (auto t : _msg->transforms)
    {
      if (!t.header.frame_id.compare("world") &&
        !t.child_frame_id.compare(this->referenceFrameID))
      {
        this->referenceFrame.Pos() = ignition::math::Vector3d(
          t.transform.translation.x,
          t.transform.translation.y,
          t.transform.translation.z);

        this->referenceFrame.Rot() = ignition::math::Quaterniond(
          t.transform.rotation.w,
          t.transform.rotation.x,
          t.transform.rotation.y,
          t.transform.rotation.z);
        this->isReferenceInit = true;
      }
    }
  }
}

/////////////////////////////////////////////////
bool ROSBasePlugin::ChangeSensorState(
    uuv_sensor_ros_plugins_msgs::ChangeSensorState::Request& _req,
    uuv_sensor_ros_plugins_msgs::ChangeSensorState::Response& _res)
{
  this->isOn.data = _req.on;
  _res.success = true;
  std::string message = this->sensorOutputTopic + "::";

  if (_req.on)
    message += " ON";
  else
    message += " OFF";
  _res.message = message;
  gzmsg << message << std::endl;
  return true;
}

/////////////////////////////////////////////////
void ROSBasePlugin::PublishState()
{
  this->pluginStatePub.publish(this->isOn);
}

/////////////////////////////////////////////////
double ROSBasePlugin::GetGaussianNoise(double _amp)
{
  return _amp * this->noiseModels["default"](this->rndGen);
}

/////////////////////////////////////////////////
double ROSBasePlugin::GetGaussianNoise(std::string _name, double _amp)
{
  GZ_ASSERT(this->noiseModels.count(_name),
    "Gaussian noise model does not exist");
  return _amp * this->noiseModels[_name](this->rndGen);
}

/////////////////////////////////////////////////
bool ROSBasePlugin::AddNoiseModel(std::string _name, double _sigma)
{
  // Check if noise model name already exists
  if (this->noiseModels.count(_name))
    return false;

  this->noiseModels[_name] = std::normal_distribution<double>(0.0, _sigma);
  return true;
}

/////////////////////////////////////////////////
bool ROSBasePlugin::IsOn()
{
  return this->isOn.data;
}

/////////////////////////////////////////////////
bool ROSBasePlugin::EnableMeasurement(const common::UpdateInfo& _info) const
{
    common::Time current_time  = _info.simTime;
    double dt = (current_time - this->lastMeasurementTime).Double();
    return dt >= 1.0 / this->updateRate && this->isReferenceInit &&
      this->isOn.data;
}

/////////////////////////////////////////////////
void ROSBasePlugin::UpdateReferenceFramePose()
{
  // Read the pose of the reference frame if it was given as a Gazebo link
  if (this->referenceLink)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->referenceFrame = this->referenceLink->WorldPose();
#else
    this->referenceFrame = this->referenceLink->GetWorldPose().Ign();
#endif
  }
}

}
