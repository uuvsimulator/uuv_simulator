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
//
// This source code is derived from hector_localization
//   (https://github.com/tu-darmstadt-ros-pkg/hector_localization)
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt,
// licensed under the BSD 3-Clause license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// The original code was modified to:
// - be more consistent with other sensor plugins within uuv_simulator,
// - adhere to Gazebo's coding standards.

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf/transform_datatypes.h>

#include <topic_tools/shape_shifter.h>

std::string g_odometry_topic;
std::string g_pose_topic;
std::string g_imu_topic;
std::string g_topic;
std::string g_frame_id;
std::string g_footprint_frame_id;
std::string g_position_frame_id;
std::string g_stabilized_frame_id;
std::string g_child_frame_id;

bool g_publish_roll_pitch;

std::string g_tf_prefix;

tf::TransformBroadcaster *g_transform_broadcaster;
ros::Publisher g_pose_publisher;
ros::Publisher g_euler_publisher;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void addTransform(std::vector<geometry_msgs::TransformStamped>& transforms, const tf::StampedTransform& tf)
{
  transforms.push_back(geometry_msgs::TransformStamped());
  tf::transformStampedTFToMsg(tf, transforms.back());
}

void sendTransform(geometry_msgs::Pose const &pose, const std_msgs::Header& header, std::string child_frame_id = "")
{
  std::vector<geometry_msgs::TransformStamped> transforms;

  tf::StampedTransform tf;
  tf.stamp_ = header.stamp;

  tf.frame_id_ = header.frame_id;
  if (!g_frame_id.empty()) tf.frame_id_ = g_frame_id;
  tf.frame_id_ = tf::resolve(g_tf_prefix, tf.frame_id_);

  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.orientation, orientation);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  tf::Point position;
  tf::pointMsgToTF(pose.position, position);

  // position intermediate transform (x,y,z)
  if( !g_position_frame_id.empty() && child_frame_id != g_position_frame_id) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, g_position_frame_id);
    tf.setOrigin(tf::Vector3(position.x(), position.y(), position.z() ));
    tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    addTransform(transforms, tf);
  }

  // footprint intermediate transform (x,y,yaw)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_footprint_frame_id) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, g_footprint_frame_id);
    tf.setOrigin(tf::Vector3(position.x(), position.y(), 0.0));
    tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
    addTransform(transforms, tf);

    yaw = 0.0;
    position.setX(0.0);
    position.setY(0.0);
    tf.frame_id_ = tf::resolve(g_tf_prefix, g_footprint_frame_id);
  }

  // stabilized intermediate transform (z)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_stabilized_frame_id) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, g_stabilized_frame_id);
    tf.setOrigin(tf::Vector3(0.0, 0.0, position.z()));
    tf.setBasis(tf::Matrix3x3::getIdentity());
    addTransform(transforms, tf);

    position.setZ(0.0);
    tf.frame_id_ = tf::resolve(g_tf_prefix, g_stabilized_frame_id);
  }

  // base_link transform (roll, pitch)
  if (g_publish_roll_pitch) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, child_frame_id);
    tf.setOrigin(position);
    tf.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
    addTransform(transforms, tf);
  }

  g_transform_broadcaster->sendTransform(transforms);

  // publish pose message
  if (g_pose_publisher) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header = header;
    g_pose_publisher.publish(pose_stamped);
  }

  // publish pose message
  if (g_euler_publisher) {
    geometry_msgs::Vector3Stamped euler_stamped;
    euler_stamped.vector.x = roll;
    euler_stamped.vector.y = pitch;
    euler_stamped.vector.z = yaw;
    euler_stamped.header = header;
    g_euler_publisher.publish(euler_stamped);
  }
}

void odomCallback(nav_msgs::Odometry const &odometry) {
  sendTransform(odometry.pose.pose, odometry.header, odometry.child_frame_id);
}

void poseCallback(geometry_msgs::PoseStamped const &pose) {
  sendTransform(pose.pose, pose.header);
}

void tfCallback(geometry_msgs::TransformStamped const &tf) {
  geometry_msgs::Pose pose;
  pose.position.x = tf.transform.translation.x;
  pose.position.y = tf.transform.translation.y;
  pose.position.z = tf.transform.translation.z;
  pose.orientation = tf.transform.rotation;

  sendTransform(pose, tf.header);
}

void imuCallback(sensor_msgs::Imu const &imu) {
  std::vector<geometry_msgs::TransformStamped> transforms;
  std::string child_frame_id;

  tf::StampedTransform tf;
  tf.stamp_ = imu.header.stamp;

  tf.frame_id_ = tf::resolve(g_tf_prefix, g_stabilized_frame_id);
  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imu.orientation, orientation);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  tf::Quaternion rollpitch = tf::createQuaternionFromRPY(roll, pitch, 0.0);

  // base_link transform (roll, pitch)
  if (g_publish_roll_pitch) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, child_frame_id);
    tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf.setRotation(rollpitch);
    addTransform(transforms, tf);
  }

  if (!transforms.empty()) g_transform_broadcaster->sendTransform(transforms);

  // publish pose message
  if (g_pose_publisher) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = imu.header.stamp;
    pose_stamped.header.frame_id = g_stabilized_frame_id;
    tf::quaternionTFToMsg(rollpitch, pose_stamped.pose.orientation);
    g_pose_publisher.publish(pose_stamped);
  }
}

void multiCallback(topic_tools::ShapeShifter const &input) {
  if (input.getDataType() == "nav_msgs/Odometry") {
    nav_msgs::Odometry::ConstPtr odom = input.instantiate<nav_msgs::Odometry>();
    odomCallback(*odom);
    return;
  }

  if (input.getDataType() == "geometry_msgs/PoseStamped") {
    geometry_msgs::PoseStamped::ConstPtr pose = input.instantiate<geometry_msgs::PoseStamped>();
    poseCallback(*pose);
    return;
  }

  if (input.getDataType() == "sensor_msgs/Imu") {
    sensor_msgs::Imu::ConstPtr imu = input.instantiate<sensor_msgs::Imu>();
    imuCallback(*imu);
    return;
  }

  if (input.getDataType() == "geometry_msgs/TransformStamped") {
    geometry_msgs::TransformStamped::ConstPtr tf = input.instantiate<geometry_msgs::TransformStamped>();
    tfCallback(*tf);
    return;
  }

  ROS_ERROR_THROTTLE(1.0, "message_to_tf received a %s message. Supported message types: nav_msgs/Odometry geometry_msgs/PoseStamped geometry_msgs/TransformStamped sensor_msgs/Imu", input.getDataType().c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "message_to_tf");

  g_footprint_frame_id = "base_footprint";
  g_stabilized_frame_id = "base_stabilized";
  // g_position_frame_id = "base_position";
  // g_child_frame_id = "base_link";

  ros::NodeHandle priv_nh("~");
  priv_nh.getParam("odometry_topic", g_odometry_topic);
  priv_nh.getParam("pose_topic", g_pose_topic);
  priv_nh.getParam("imu_topic", g_imu_topic);
  priv_nh.getParam("topic", g_topic);
  priv_nh.getParam("frame_id", g_frame_id);
  priv_nh.getParam("footprint_frame_id", g_footprint_frame_id);
  priv_nh.getParam("position_frame_id", g_position_frame_id);
  priv_nh.getParam("stabilized_frame_id", g_stabilized_frame_id);
  priv_nh.getParam("child_frame_id", g_child_frame_id);

  // get topic from the commandline
  if (argc > 1) {
      g_topic = argv[1];
      g_odometry_topic.clear();
      g_pose_topic.clear();
      g_imu_topic.clear();
  }

  g_publish_roll_pitch = true;
  priv_nh.getParam("publish_roll_pitch", g_publish_roll_pitch);

  g_tf_prefix = tf::getPrefixParam(priv_nh);
  g_transform_broadcaster = new tf::TransformBroadcaster;

  ros::NodeHandle node;
  ros::Subscriber sub1, sub2, sub3, sub4;
  int subscribers = 0;
  if (!g_odometry_topic.empty()) {
      sub1 = node.subscribe(g_odometry_topic, 10, &odomCallback);
      subscribers++;
  }
  if (!g_pose_topic.empty()) {
      sub2 = node.subscribe(g_pose_topic, 10, &poseCallback);
      subscribers++;
  }
  if (!g_imu_topic.empty()) {
      sub3 = node.subscribe(g_imu_topic, 10, &imuCallback);
      subscribers++;
  }
  if (!g_topic.empty()) {
      sub4 = node.subscribe(g_topic, 10, &multiCallback);
      subscribers++;
  }

  if (subscribers == 0) {
    ROS_FATAL("Usage: rosrun message_to_tf message_to_tf <topic>");
    return 1;
  } else if (subscribers > 1) {
    ROS_FATAL("More than one of the parameters odometry_topic, pose_topic, imu_topic and topic are set.\n"
              "Please specify exactly one of them or simply add the topic name to the command line.");
    return 1;
  }

  bool publish_pose = true;
  priv_nh.getParam("publish_pose", publish_pose);
  if (publish_pose) {
    std::string publish_pose_topic;
    priv_nh.getParam("publish_pose_topic", publish_pose_topic);

    if (!publish_pose_topic.empty())
      g_pose_publisher = node.advertise<geometry_msgs::PoseStamped>(publish_pose_topic, 10);
    else
      g_pose_publisher = priv_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
  }

  bool publish_euler = true;
  priv_nh.getParam("publish_euler", publish_euler);
  if (publish_euler) {
    std::string publish_euler_topic;
    priv_nh.getParam("publish_euler_topic", publish_euler_topic);

    if (!publish_euler_topic.empty())
      g_euler_publisher = node.advertise<geometry_msgs::Vector3Stamped>(publish_euler_topic, 10);
    else
      g_euler_publisher = priv_nh.advertise<geometry_msgs::Vector3Stamped>("euler", 10);
  }

  ros::spin();
  delete g_transform_broadcaster;
  return 0;
}
