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
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Init()
{
  UnderwaterObjectPlugin::Init();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Reset()
{
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

    gazebo::math::Vector3 restoring = this->models[_link]->GetStoredVector(
      RESTORING_FORCE);

    geometry_msgs::WrenchStamped msg;
    this->GenWrenchMsg(restoring,
      gazebo::math::Vector3(0, 0, 0), msg);
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
  if (this->flowVelocity.GetLength() > 0)
  {
    marker.action = visualization_msgs::Marker::ADD;

    gazebo::math::Pose pose = this->model->GetWorldPose();

    double yaw = std::atan2(this->flowVelocity.y, this->flowVelocity.x);
    double pitch = std::atan2(
      this->flowVelocity.z,
      std::sqrt(std::pow(this->flowVelocity.x, 2) +
        std::pow(this->flowVelocity.y, 2)));

    gazebo::math::Quaternion qt(0.0, -pitch, yaw);
    marker.pose.position.x = pose.pos.x;
    marker.pose.position.y = pose.pos.y;
    marker.pose.position.z = pose.pos.z + 1.5;
    marker.pose.orientation.x = qt.x;
    marker.pose.orientation.y = qt.y;
    marker.pose.orientation.z = qt.z;
    marker.pose.orientation.w = qt.w;
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
    gazebo::math::Vector3 force, torque;

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
  gazebo::math::Vector3 _force, gazebo::math::Vector3 _torque,
  geometry_msgs::WrenchStamped &_output)
{
  _output.wrench.force.x = _force.x;
  _output.wrench.force.y = _force.y;
  _output.wrench.force.z = _force.z;

  _output.wrench.torque.x = _torque.x;
  _output.wrench.torque.y = _torque.y;
  _output.wrench.torque.z = _torque.z;

  _output.header.stamp = ros::Time(this->world->GetSimTime().Double());
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::UpdateLocalCurrentVelocity(
  const geometry_msgs::Vector3::ConstPtr &_msg)
{
  if (!this->useGlobalCurrent)
  {
    this->flowVelocity.x = _msg->x;
    this->flowVelocity.y = _msg->y;
    this->flowVelocity.z = _msg->z;
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
    this->flowVelocity.x = 0;
    this->flowVelocity.y = 0;
    this->flowVelocity.z = 0;
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

    _res.models.push_back(model);
  }
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(UnderwaterObjectROSPlugin)
}
