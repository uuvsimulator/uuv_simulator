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

#include <uuv_gazebo_ros_plugins/UnderwaterObjectROSPlugin.hh>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>

namespace uuv_simulator_ros
{
/////////////////////////////////////////////////
UnderwaterObjectROSPlugin::UnderwaterObjectROSPlugin()
{
}

/////////////////////////////////////////////////
UnderwaterObjectROSPlugin::~UnderwaterObjectROSPlugin()
{
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Load(gazebo::physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->rosNode.reset(new ros::NodeHandle(""));

  try
  {
    UnderwaterObjectPlugin::Load(_parent, _sdf);
  }
  catch(gazebo::common::Exception &_e)
  {
    gzerr << "Error loading plugin."
          << "Please ensure that your model is correct."
          << '\n';
    return;
  }

  this->subLocalCurVel = this->rosNode->subscribe<geometry_msgs::Vector3>(
    _parent->GetName() + "/current_velocity", 10,
    boost::bind(&UnderwaterObjectROSPlugin::UpdateLocalCurrentVelocity,
    this, _1));

  this->services["set_use_global_current_velocity"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/set_use_global_current_velocity",
      &UnderwaterObjectROSPlugin::SetUseGlobalCurrentVel, this);

  this->services["set_added_mass_scaling"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/set_added_mass_scaling",
      &UnderwaterObjectROSPlugin::SetScalingAddedMass, this);

  this->services["get_added_mass_scaling"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/get_added_mass_scaling",
      &UnderwaterObjectROSPlugin::GetScalingAddedMass, this);

  this->services["set_damping_scaling"] =
    this->rosNode->advertiseService(
        _parent->GetName() + "/set_damping_scaling",
        &UnderwaterObjectROSPlugin::SetScalingDamping, this);

  this->services["get_damping_scaling"] =
    this->rosNode->advertiseService(
        _parent->GetName() + "/get_damping_scaling",
        &UnderwaterObjectROSPlugin::GetScalingDamping, this);

  this->services["set_volume_scaling"] =
    this->rosNode->advertiseService(
        _parent->GetName() + "/set_volume_scaling",
        &UnderwaterObjectROSPlugin::SetScalingVolume, this);

  this->services["get_volume_scaling"] =
    this->rosNode->advertiseService(
        _parent->GetName() + "/get_volume_scaling",
        &UnderwaterObjectROSPlugin::GetScalingVolume, this);

  this->services["set_fluid_density"] =
    this->rosNode->advertiseService(
        _parent->GetName() + "/set_fluid_density",
        &UnderwaterObjectROSPlugin::SetFluidDensity, this);

  this->services["get_fluid_density"] =
    this->rosNode->advertiseService(
        _parent->GetName() + "/get_fluid_density",
        &UnderwaterObjectROSPlugin::GetFluidDensity, this);

  this->services["set_volume_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/set_volume_offset",
      &UnderwaterObjectROSPlugin::SetOffsetVolume, this);

  this->services["get_volume_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/get_volume_offset",
      &UnderwaterObjectROSPlugin::GetOffsetVolume, this);

  this->services["set_added_mass_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/set_added_mass_offset",
      &UnderwaterObjectROSPlugin::SetOffsetAddedMass, this);

  this->services["get_added_mass_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/get_added_mass_offset",
      &UnderwaterObjectROSPlugin::GetOffsetAddedMass, this);

  this->services["set_linear_damping_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/set_linear_damping_offset",
      &UnderwaterObjectROSPlugin::SetOffsetLinearDamping, this);

  this->services["get_linear_damping_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/get_linear_damping_offset",
      &UnderwaterObjectROSPlugin::GetOffsetLinearDamping, this);

  this->services["set_linear_forward_speed_damping_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/set_linear_forward_speed_damping_offset",
      &UnderwaterObjectROSPlugin::SetOffsetLinearForwardSpeedDamping, this);

  this->services["get_linear_forward_speed_damping_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/get_linear_forward_speed_damping_offset",
      &UnderwaterObjectROSPlugin::GetOffsetLinearForwardSpeedDamping, this);

  this->services["set_nonlinear_damping_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/set_nonlinear_damping_offset",
      &UnderwaterObjectROSPlugin::SetOffsetNonLinearDamping, this);

  this->services["get_nonlinear_damping_offset"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/get_nonlinear_damping_offset",
      &UnderwaterObjectROSPlugin::GetOffsetNonLinearDamping, this);

  this->services["get_model_properties"] =
    this->rosNode->advertiseService(
      _parent->GetName() + "/get_model_properties",
      &UnderwaterObjectROSPlugin::GetModelProperties, this);

  this->rosHydroPub["current_velocity_marker"] =
    this->rosNode->advertise<visualization_msgs::Marker>
    (_parent->GetName() + "/current_velocity_marker", 0);

  this->rosHydroPub["using_global_current_velocity"] =
    this->rosNode->advertise<std_msgs::Bool>
    (_parent->GetName() + "/using_global_current_velocity", 0);

  this->rosHydroPub["is_submerged"] =
    this->rosNode->advertise<std_msgs::Bool>
    (_parent->GetName() + "/is_submerged", 0);

  this->nedTransform.header.frame_id = this->model->GetName() + "/base_link";
  this->nedTransform.child_frame_id = this->model->GetName() + "/base_link_ned";
  this->nedTransform.transform.translation.x = 0;
  this->nedTransform.transform.translation.y = 0;
  this->nedTransform.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, 0);
  this->nedTransform.transform.rotation.x = quat.x();
  this->nedTransform.transform.rotation.y = quat.y();
  this->nedTransform.transform.rotation.z = quat.z();
  this->nedTransform.transform.rotation.w = quat.w();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Init()
{
  UnderwaterObjectPlugin::Init();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Reset()
{ }

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  UnderwaterObjectPlugin::Update(_info);

  this->nedTransform.header.stamp = ros::Time::now();
  this->tfBroadcaster.sendTransform(this->nedTransform);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::InitDebug(gazebo::physics::LinkPtr _link,
  gazebo::HydrodynamicModelPtr _hydro)
{
  UnderwaterObjectPlugin::InitDebug(_link, _hydro);

  // Publish the stamped wrench topics if the debug flag is on
  for (std::map<std::string,
    gazebo::transport::PublisherPtr>::iterator it = this->hydroPub.begin();
    it != this->hydroPub.end(); ++it)
  {
    this->rosHydroPub[it->first] =
      this->rosNode->advertise<geometry_msgs::WrenchStamped>(
        it->second->GetTopic(), 10);
      gzmsg << "ROS TOPIC: " << it->second->GetTopic() << std::endl;
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishRestoringForce(
  gazebo::physics::LinkPtr _link)
{
  // Call base class method
  UnderwaterObjectPlugin::PublishRestoringForce(_link);

  // Publish data in a ROS topic
  if (this->models.count(_link))
  {
    if (!this->models[_link]->GetDebugFlag())
      return;

    ignition::math::Vector3d restoring = this->models[_link]->GetStoredVector(
      RESTORING_FORCE);

    geometry_msgs::WrenchStamped msg;
    this->GenWrenchMsg(restoring,
      ignition::math::Vector3d(0, 0, 0), msg);
    this->rosHydroPub[_link->GetName() + "/restoring"].publish(msg);
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishIsSubmerged()
{
  if (this->baseLinkName.empty())
    gzwarn << "Base link name string is empty" << std::endl;
  std_msgs::Bool isSubmerged;
  isSubmerged.data = this->models[this->model->GetLink(this->baseLinkName)]->IsSubmerged();
  this->rosHydroPub["is_submerged"].publish(isSubmerged);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishCurrentVelocityMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = this->model->GetName() + "/current_velocity_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  // Creating the arrow marker for the current velocity information
  // (orientation only, magnitude has to be read from the topic)
  if (this->flowVelocity.Length() > 0)
  {
    marker.action = visualization_msgs::Marker::ADD;
    ignition::math::Pose3d pose;
#if GAZEBO_MAJOR_VERSION >= 8
    pose = this->model->WorldPose();
#else
    pose = this->model->GetWorldPose().Ign();
#endif
    double yaw = std::atan2(this->flowVelocity.Y(), this->flowVelocity.X());
    double pitch = std::atan2(
      this->flowVelocity.Z(),
      std::sqrt(std::pow(this->flowVelocity.X(), 2) +
        std::pow(this->flowVelocity.Y(), 2)));

    ignition::math::Quaterniond qt(0.0, -pitch, yaw);
    marker.pose.position.x = pose.Pos().X();
    marker.pose.position.y = pose.Pos().Y();
    marker.pose.position.z = pose.Pos().Z() + 1.5;
    marker.pose.orientation.x = qt.X();
    marker.pose.orientation.y = qt.Y();
    marker.pose.orientation.z = qt.Z();
    marker.pose.orientation.w = qt.W();
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
  else
  {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  // Publish current velocity RViz marker
  this->rosHydroPub["current_velocity_marker"].publish(marker);
  // Publishing flag for usage of global current velocity
  std_msgs::Bool useGlobalMsg;
  useGlobalMsg.data = this->useGlobalCurrent;
  this->rosHydroPub["using_global_current_velocity"].publish(useGlobalMsg);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishHydrodynamicWrenches(
  gazebo::physics::LinkPtr _link)
{
  // Call base class method
  UnderwaterObjectPlugin::PublishRestoringForce(_link);

  // Publish data in a ROS topic
  if (this->models.count(_link))
  {
    if (!this->models[_link]->GetDebugFlag())
      return;
    geometry_msgs::WrenchStamped msg;
    ignition::math::Vector3d force, torque;

    // Publish wrench generated by the acceleration of fluid around the object
    force = this->models[_link]->GetStoredVector(UUV_ADDED_MASS_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_ADDED_MASS_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    this->rosHydroPub[_link->GetName() + "/added_mass"].publish(msg);

    // Publish wrench generated by the fluid damping
    force = this->models[_link]->GetStoredVector(UUV_DAMPING_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_DAMPING_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    this->rosHydroPub[_link->GetName() + "/damping"].publish(msg);

    // Publish wrench generated by the Coriolis forces
    force = this->models[_link]->GetStoredVector(UUV_ADDED_CORIOLIS_FORCE);
    torque = this->models[_link]->GetStoredVector(UUV_ADDED_CORIOLIS_TORQUE);

    this->GenWrenchMsg(force, torque, msg);
    this->rosHydroPub[_link->GetName() + "/added_coriolis"].publish(msg);
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GenWrenchMsg(
  ignition::math::Vector3d _force, ignition::math::Vector3d _torque,
  geometry_msgs::WrenchStamped &_output)
{
  _output.wrench.force.x = _force.X();
  _output.wrench.force.y = _force.Y();
  _output.wrench.force.z = _force.Z();

  _output.wrench.torque.x = _torque.X();
  _output.wrench.torque.y = _torque.Y();
  _output.wrench.torque.z = _torque.Z();
#if GAZEBO_MAJOR_VERSION >= 8
  _output.header.stamp = ros::Time(this->world->SimTime().Double());
#else
  _output.header.stamp = ros::Time(this->world->GetSimTime().Double());
#endif
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::UpdateLocalCurrentVelocity(
  const geometry_msgs::Vector3::ConstPtr &_msg)
{
  if (!this->useGlobalCurrent)
  {
    this->flowVelocity.X() = _msg->x;
    this->flowVelocity.Y() = _msg->y;
    this->flowVelocity.Z() = _msg->z;
  }
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetUseGlobalCurrentVel(
  uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel::Response& _res)
{
  if (_req.use_global == this->useGlobalCurrent)
    _res.success = true;
  else
  {
    this->useGlobalCurrent = _req.use_global;
    this->flowVelocity.X() = 0;
    this->flowVelocity.Y() = 0;
    this->flowVelocity.Z() = 0;
    if (this->useGlobalCurrent)
      gzmsg << this->model->GetName() <<
        "::Now using global current velocity" << std::endl;
    else
      gzmsg << this->model->GetName() <<
        "::Using the current velocity under the namespace " <<
        this->model->GetName() << std::endl;
    _res.success = true;
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetModelProperties(
  uuv_gazebo_ros_plugins_msgs::GetModelProperties::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetModelProperties::Response& _res)
{
  for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
  {
    gazebo::physics::LinkPtr link = it->first;
    gazebo::HydrodynamicModelPtr hydro = it->second;

    _res.link_names.push_back(link->GetName());

    uuv_gazebo_ros_plugins_msgs::UnderwaterObjectModel model;
    double param;
    std::vector<double> mat;

    hydro->GetParam("volume", param);
    model.volume = param;

    hydro->GetParam("fluid_density", param);
    model.fluid_density = param;

    hydro->GetParam("bbox_height", param);
    model.bbox_height = param;

    hydro->GetParam("bbox_length", param);
    model.bbox_length = param;

    hydro->GetParam("bbox_width", param);
    model.bbox_width = param;

    hydro->GetParam("added_mass", mat);
    model.added_mass = mat;

    hydro->GetParam("linear_damping", mat);
    model.linear_damping = mat;

    hydro->GetParam("linear_damping_forward_speed", mat);
    model.linear_damping_forward_speed = mat;

    hydro->GetParam("quadratic_damping", mat);
    model.quadratic_damping = mat;

    model.neutrally_buoyant = hydro->IsNeutrallyBuoyant();

    hydro->GetParam("center_of_buoyancy", mat);
    model.cob.x = mat[0];
    model.cob.y = mat[1];
    model.cob.z = mat[2];
#if GAZEBO_MAJOR_VERSION >= 8
    model.inertia.m = link->GetInertial()->Mass();
    model.inertia.ixx = link->GetInertial()->IXX();
    model.inertia.ixy = link->GetInertial()->IXY();
    model.inertia.ixz = link->GetInertial()->IXZ();
    model.inertia.iyy = link->GetInertial()->IYY();
    model.inertia.iyz = link->GetInertial()->IYZ();
    model.inertia.izz = link->GetInertial()->IZZ();

    model.inertia.com.x = link->GetInertial()->CoG().X();
    model.inertia.com.y = link->GetInertial()->CoG().Y();
    model.inertia.com.z = link->GetInertial()->CoG().Z();
#else
    model.inertia.m = link->GetInertial()->GetMass();
    model.inertia.ixx = link->GetInertial()->GetIXX();
    model.inertia.ixy = link->GetInertial()->GetIXY();
    model.inertia.ixz = link->GetInertial()->GetIXZ();
    model.inertia.iyy = link->GetInertial()->GetIYY();
    model.inertia.iyz = link->GetInertial()->GetIYZ();
    model.inertia.izz = link->GetInertial()->GetIZZ();

    model.inertia.com.x = link->GetInertial()->GetCoG().x;
    model.inertia.com.y = link->GetInertial()->GetCoG().y;
    model.inertia.com.z = link->GetInertial()->GetCoG().z;
#endif
    _res.models.push_back(model);
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetScalingAddedMass(
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetFloat::Response& _res)

{
  if (_req.data < 0)
  {
    _res.success = false;
    _res.message = "Scaling factor cannot be negative";
  }
  else
  {
    for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
    {
      gazebo::HydrodynamicModelPtr hydro = it->second;
      hydro->SetParam("scaling_added_mass", _req.data);
    }
    _res.success = true;
    _res.message = "All links set with new added-mass scaling factor";
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetScalingAddedMass(
  uuv_gazebo_ros_plugins_msgs::GetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetFloat::Response& _res)

{
  models.begin()->second->GetParam("scaling_added_mass", _res.data);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetScalingDamping(
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetFloat::Response& _res)

{
  if (_req.data < 0)
  {
    _res.success = false;
    _res.message = "Scaling factor cannot be negative";
  }
  else
  {
    for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
    {
      gazebo::HydrodynamicModelPtr hydro = it->second;
      hydro->SetParam("scaling_damping", _req.data);
    }
    _res.success = true;
    _res.message = "All links set with new damping scaling factor";
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetScalingDamping(
  uuv_gazebo_ros_plugins_msgs::GetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetFloat::Response& _res)

{
  models.begin()->second->GetParam("scaling_damping", _res.data);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetScalingVolume(
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetFloat::Response& _res)

{
  if (_req.data < 0)
  {
    _res.success = false;
    _res.message = "Scaling factor cannot be negative";
  }
  else
  {
    for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
    {
      gazebo::HydrodynamicModelPtr hydro = it->second;
      hydro->SetParam("scaling_volume", _req.data);
    }
    _res.success = true;
    _res.message = "All links set with new volume scaling factor";
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetScalingVolume(
  uuv_gazebo_ros_plugins_msgs::GetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetFloat::Response& _res)

{
  models.begin()->second->GetParam("scaling_volume", _res.data);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetFluidDensity(
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetFloat::Response& _res)

{
  if (_req.data < 0)
  {
    _res.success = false;
    _res.message = "Scaling factor cannot be negative";
  }
  else
  {
    for (std::map<gazebo::physics::LinkPtr,
       gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
       it != models.end(); ++it)
    {
      gazebo::HydrodynamicModelPtr hydro = it->second;
      hydro->SetParam("fluid_density", _req.data);
    }
    _res.success = true;
    _res.message = "All links set with new fluid density";
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetFluidDensity(
  uuv_gazebo_ros_plugins_msgs::GetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetFloat::Response& _res)

{
  models.begin()->second->GetParam("fluid_density", _res.data);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetOffsetVolume(
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetFloat::Response& _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_volume", _req.data);
  }
  _res.success = true;
  _res.message = "All links set with new volume offset";

  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetOffsetVolume(
  uuv_gazebo_ros_plugins_msgs::GetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetFloat::Response& _res)

{
  models.begin()->second->GetParam("offset_volume", _res.data);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetOffsetAddedMass(
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetFloat::Response& _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_added_mass", _req.data);
  }
  _res.success = true;
  _res.message = "All links set with new added-mass identity offset";

  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetOffsetAddedMass(
  uuv_gazebo_ros_plugins_msgs::GetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetFloat::Response& _res)

{
  models.begin()->second->GetParam("offset_added_mass", _res.data);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetOffsetLinearDamping(
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetFloat::Response& _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_linear_damping", _req.data);
  }
  _res.success = true;
  _res.message = "All links set with new linear damping identity offset";

  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetOffsetLinearDamping(
  uuv_gazebo_ros_plugins_msgs::GetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetFloat::Response& _res)

{
  models.begin()->second->GetParam("offset_linear_damping", _res.data);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetOffsetLinearForwardSpeedDamping(
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetFloat::Response& _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_lin_forward_speed_damping", _req.data);
  }
  _res.success = true;
  _res.message = "All links set with new linear forward speed damping identity offset";

  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetOffsetLinearForwardSpeedDamping(
  uuv_gazebo_ros_plugins_msgs::GetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetFloat::Response& _res)

{
  models.begin()->second->GetParam("offset_lin_forward_speed_damping", _res.data);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetOffsetNonLinearDamping(
  uuv_gazebo_ros_plugins_msgs::SetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::SetFloat::Response& _res)

{
  for (std::map<gazebo::physics::LinkPtr,
     gazebo::HydrodynamicModelPtr>::iterator it = models.begin();
     it != models.end(); ++it)
  {
    gazebo::HydrodynamicModelPtr hydro = it->second;
    hydro->SetParam("offset_nonlin_damping", _req.data);
  }
  _res.success = true;
  _res.message = "All links set with new nonlinear damping identity offset";

  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetOffsetNonLinearDamping(
  uuv_gazebo_ros_plugins_msgs::GetFloat::Request& _req,
  uuv_gazebo_ros_plugins_msgs::GetFloat::Response& _res)

{
  models.begin()->second->GetParam("offset_nonlin_damping", _res.data);
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(UnderwaterObjectROSPlugin)
}
