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

#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>

namespace gazebo
{

/////////////////////////////////////////////////
ROSBaseModelPlugin::ROSBaseModelPlugin()
{
  // Initialize local NED frame
  this->localNEDFrame = ignition::math::Pose3d::Zero;
  this->localNEDFrame.Rot() = ignition::math::Quaterniond(
    ignition::math::Vector3d(M_PI, 0, 0));
  // Initialize the local NED frame
  this->tfLocalNEDFrame.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  this->tfLocalNEDFrame.setRotation(
    tf::createQuaternionFromRPY(M_PI, 0.0, 0.0));
  // Initialize TF broadcaster
  this->tfBroadcaster = new tf::TransformBroadcaster();
}

/////////////////////////////////////////////////
ROSBaseModelPlugin::~ROSBaseModelPlugin()
{
  if (this->tfBroadcaster)
    delete this->tfBroadcaster;
}

/////////////////////////////////////////////////
void ROSBaseModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize model pointer
  this->model = _model;

  // Store world pointer
  this->world = this->model->GetWorld();

  std::string linkName;
  GZ_ASSERT(_sdf->HasElement("link_name"), "No link name provided");
  GetSDFParam<std::string>(_sdf, "link_name", linkName, "");
  GZ_ASSERT(!linkName.empty(), "Link name string is empty");

  // Get flag to enable generation of Gazebo messages
  GetSDFParam<bool>(_sdf, "enable_local_ned_frame", this->enableLocalNEDFrame,
    true);

  if (_sdf->HasElement("reference_link_name"))
  {
    std::string refLinkName;
    GetSDFParam<std::string>(_sdf, "reference_link_name", refLinkName, "");
    if (!refLinkName.empty())
    {
      this->referenceLink = this->model->GetLink(refLinkName);
      GZ_ASSERT(this->referenceLink != NULL, "Invalid reference link");
      this->referenceFrameID = refLinkName;
    }
  }

  // Get sensor link
  this->link = this->model->GetLink(linkName);
  GZ_ASSERT(this->link != NULL, "Invalid link pointer");

  // Set the frame IDs for the local NED frame
  this->tfLocalNEDFrame.frame_id_ = this->link->GetName();
  this->tfLocalNEDFrame.child_frame_id_ = this->link->GetName() + "_ned";

  this->InitBasePlugin(_sdf);

  // Bind the sensor update callback function to the world update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ROSBasePlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
bool ROSBaseModelPlugin::OnUpdate(const common::UpdateInfo&)
{
  return true;
}

/////////////////////////////////////////////////
void ROSBaseModelPlugin::SendLocalNEDTransform()
{
  geometry_msgs::TransformStamped msg;
  this->tfLocalNEDFrame.stamp_ = ros::Time::now();
  tf::transformStampedTFToMsg(this->tfLocalNEDFrame, msg);
  this->tfBroadcaster->sendTransform(msg);  
}

}
