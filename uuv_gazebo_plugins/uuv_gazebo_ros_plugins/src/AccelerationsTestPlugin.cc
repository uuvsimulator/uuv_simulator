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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Vector3.hh>

#include <ros/ros.h>
#include <geometry_msgs/Accel.h>

#include <uuv_gazebo_ros_plugins/AccelerationsTestPlugin.hh>
#include <uuv_gazebo_plugins/Def.hh>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(AccelerationsTestPlugin)

/////////////////////////////////////////////////
AccelerationsTestPlugin::AccelerationsTestPlugin()
{
}

/////////////////////////////////////////////////
AccelerationsTestPlugin::~AccelerationsTestPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void AccelerationsTestPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Invalid model pointer");
  GZ_ASSERT(_sdf != NULL, "Invalid SDF element pointer");

  this->model = _model;
  this->world = _model->GetWorld();

  // Initialize the transport node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  std::string link_name;
  if (_sdf->HasElement("link_name"))
    link_name = _sdf->GetElement("link_name")->Get<std::string>();
  else
  gzerr << "[TestPlugin] Please specify a link_name .\n";
  this->link = this->model->GetLink(link_name);
  if (this->link == NULL)
    gzthrow("[TestPlugin] Could not find specified link \""
      << link_name << "\".");

  // Connect the update event callback
  this->Connect();

  // ROS:
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->rosNode.reset(new ros::NodeHandle(""));

  this->pub_accel_w_gazebo =
    this->rosNode->advertise<geometry_msgs::Accel>("accel_w_gazebo", 10);
  this->pub_accel_w_numeric =
    this->rosNode->advertise<geometry_msgs::Accel>("accel_w_numeric", 10);

  this->pub_accel_b_gazebo =
    this->rosNode->advertise<geometry_msgs::Accel>("accel_b_gazebo", 10);
  this->pub_accel_b_numeric =
    this->rosNode->advertise<geometry_msgs::Accel>("accel_b_numeric", 10);
}

/////////////////////////////////////////////////
void AccelerationsTestPlugin::Init()
{
  // Doing nothing for now
}

geometry_msgs::Accel accelFromEigen(const Eigen::Vector6d& acc)
{
  geometry_msgs::Accel amsg;
  amsg.linear.x = acc[0];
  amsg.linear.y = acc[1];
  amsg.linear.z = acc[2];
  amsg.angular.x = acc[3];
  amsg.angular.y = acc[4];
  amsg.angular.z = acc[5];
  return amsg;
}

Eigen::Matrix3d Matrix3ToEigen(const math::Matrix3& m)
{
  Eigen::Matrix3d r;
  r << m[0][0], m[0][1], m[0][2],
    m[1][0], m[1][1], m[1][2],
    m[2][0], m[2][1], m[2][2];
  return r;
}

/////////////////////////////////////////////////
void AccelerationsTestPlugin::Update(const common::UpdateInfo &_info)
{
  double dt = (_info.simTime - lastTime).Double();

  // Link's pose
  const math::Pose pose_w_b = this->link->GetWorldPose();

  // Velocities of this link in link frame.
  Eigen::Vector6d gazebo_b_v_w_b = EigenStack(
    this->link->GetRelativeLinearVel(),
    this->link->GetRelativeAngularVel());

  // Velocities of this link in world frame
  Eigen::Vector6d gazebo_w_v_w_b = EigenStack(
    this->link->GetWorldLinearVel(),
    this->link->GetWorldAngularVel());


  // Accelerations of this link in world frame
  Eigen::Vector6d gazebo_w_a_w_b = EigenStack(
    this->link->GetWorldLinearAccel(),
    this->link->GetWorldAngularAccel());

  // Accelerations of this link in link frame
  Eigen::Vector6d gazebo_b_a_w_b = EigenStack(
    this->link->GetRelativeLinearAccel(),
    this->link->GetRelativeAngularAccel());

  // Numerically computed accelerations
  math::Quaternion q_b_w = pose_w_b.rot.GetInverse();
  math::Matrix3 R_b_w = q_b_w.GetAsMatrix3();

  Eigen::Matrix3d R_b_w_eigen = Matrix3ToEigen(R_b_w);
  Eigen::Matrix6d R6_b_w_eigen;
  R6_b_w_eigen << R_b_w_eigen, Eigen::Matrix3d::Zero(),
                  Eigen::Matrix3d::Zero(), R_b_w_eigen;

  // Actual numeric differentiation
  Eigen::Vector6d num_w_a_w_b = (gazebo_w_v_w_b - last_w_v_w_b)/dt;
  Eigen::Vector6d num_b_a_w_b = R6_b_w_eigen*num_w_a_w_b;

  // Publish all four variants via ROS for easy comparison
  this->pub_accel_w_gazebo.publish(accelFromEigen(gazebo_w_a_w_b));
  this->pub_accel_b_gazebo.publish(accelFromEigen(gazebo_b_a_w_b));

  this->pub_accel_w_numeric.publish(accelFromEigen(num_w_a_w_b));
  this->pub_accel_b_numeric.publish(accelFromEigen(num_b_a_w_b));

  last_w_v_w_b = gazebo_w_v_w_b;
  lastTime = _info.simTime;
}

/////////////////////////////////////////////////
void AccelerationsTestPlugin::Connect()
{
  // Connect the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&AccelerationsTestPlugin::Update,
                    this, _1));
}
}
