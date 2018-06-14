; Auto-generated. Do not edit!


(cl:in-package uuv_sensor_plugins_ros_msgs-msg)


;//! \htmlinclude PositionWithCovariance.msg.html

(cl:defclass <PositionWithCovariance> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass PositionWithCovariance (<PositionWithCovariance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionWithCovariance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionWithCovariance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_sensor_plugins_ros_msgs-msg:<PositionWithCovariance> is deprecated: use uuv_sensor_plugins_ros_msgs-msg:PositionWithCovariance instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <PositionWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:pos-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <PositionWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:covariance-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:covariance instead.")
  (covariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionWithCovariance>) ostream)
  "Serializes a message object of type '<PositionWithCovariance>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionWithCovariance>) istream)
  "Deserializes a message object of type '<PositionWithCovariance>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionWithCovariance>)))
  "Returns string type for a message object of type '<PositionWithCovariance>"
  "uuv_sensor_plugins_ros_msgs/PositionWithCovariance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionWithCovariance)))
  "Returns string type for a message object of type 'PositionWithCovariance"
  "uuv_sensor_plugins_ros_msgs/PositionWithCovariance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionWithCovariance>)))
  "Returns md5sum for a message object of type '<PositionWithCovariance>"
  "4a54596b2ea1a0ed659f46ab0b26f202")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionWithCovariance)))
  "Returns md5sum for a message object of type 'PositionWithCovariance"
  "4a54596b2ea1a0ed659f46ab0b26f202")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionWithCovariance>)))
  "Returns full string definition for message of type '<PositionWithCovariance>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This represents a position in free space with uncertainty.~%~%geometry_msgs/Point pos~%~%# Row-major representation of the 3x3 covariance matrix~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionWithCovariance)))
  "Returns full string definition for message of type 'PositionWithCovariance"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This represents a position in free space with uncertainty.~%~%geometry_msgs/Point pos~%~%# Row-major representation of the 3x3 covariance matrix~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionWithCovariance>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionWithCovariance>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionWithCovariance
    (cl:cons ':pos (pos msg))
    (cl:cons ':covariance (covariance msg))
))
