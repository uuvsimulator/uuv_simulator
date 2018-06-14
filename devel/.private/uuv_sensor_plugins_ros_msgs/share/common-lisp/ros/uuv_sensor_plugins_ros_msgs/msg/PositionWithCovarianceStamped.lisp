; Auto-generated. Do not edit!


(cl:in-package uuv_sensor_plugins_ros_msgs-msg)


;//! \htmlinclude PositionWithCovarianceStamped.msg.html

(cl:defclass <PositionWithCovarianceStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pos
    :reader pos
    :initarg :pos
    :type uuv_sensor_plugins_ros_msgs-msg:PositionWithCovariance
    :initform (cl:make-instance 'uuv_sensor_plugins_ros_msgs-msg:PositionWithCovariance)))
)

(cl:defclass PositionWithCovarianceStamped (<PositionWithCovarianceStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionWithCovarianceStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionWithCovarianceStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_sensor_plugins_ros_msgs-msg:<PositionWithCovarianceStamped> is deprecated: use uuv_sensor_plugins_ros_msgs-msg:PositionWithCovarianceStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PositionWithCovarianceStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:header-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <PositionWithCovarianceStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:pos-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionWithCovarianceStamped>) ostream)
  "Serializes a message object of type '<PositionWithCovarianceStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionWithCovarianceStamped>) istream)
  "Deserializes a message object of type '<PositionWithCovarianceStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionWithCovarianceStamped>)))
  "Returns string type for a message object of type '<PositionWithCovarianceStamped>"
  "uuv_sensor_plugins_ros_msgs/PositionWithCovarianceStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionWithCovarianceStamped)))
  "Returns string type for a message object of type 'PositionWithCovarianceStamped"
  "uuv_sensor_plugins_ros_msgs/PositionWithCovarianceStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionWithCovarianceStamped>)))
  "Returns md5sum for a message object of type '<PositionWithCovarianceStamped>"
  "ef0ae60585e532ef356441a1a701f864")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionWithCovarianceStamped)))
  "Returns md5sum for a message object of type 'PositionWithCovarianceStamped"
  "ef0ae60585e532ef356441a1a701f864")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionWithCovarianceStamped>)))
  "Returns full string definition for message of type '<PositionWithCovarianceStamped>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This expresses an estimated position with a reference coordinate frame and~%# timestamp~%~%Header header~%PositionWithCovariance pos~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: uuv_sensor_plugins_ros_msgs/PositionWithCovariance~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This represents a position in free space with uncertainty.~%~%geometry_msgs/Point pos~%~%# Row-major representation of the 3x3 covariance matrix~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionWithCovarianceStamped)))
  "Returns full string definition for message of type 'PositionWithCovarianceStamped"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This expresses an estimated position with a reference coordinate frame and~%# timestamp~%~%Header header~%PositionWithCovariance pos~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: uuv_sensor_plugins_ros_msgs/PositionWithCovariance~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This represents a position in free space with uncertainty.~%~%geometry_msgs/Point pos~%~%# Row-major representation of the 3x3 covariance matrix~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionWithCovarianceStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionWithCovarianceStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionWithCovarianceStamped
    (cl:cons ':header (header msg))
    (cl:cons ':pos (pos msg))
))
