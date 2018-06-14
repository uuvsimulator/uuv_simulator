; Auto-generated. Do not edit!


(cl:in-package uuv_sensor_plugins_ros_msgs-msg)


;//! \htmlinclude ChemicalParticleConcentration.msg.html

(cl:defclass <ChemicalParticleConcentration> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0)
   (concentration
    :reader concentration
    :initarg :concentration
    :type cl:float
    :initform 0.0)
   (is_measuring
    :reader is_measuring
    :initarg :is_measuring
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ChemicalParticleConcentration (<ChemicalParticleConcentration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChemicalParticleConcentration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChemicalParticleConcentration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_sensor_plugins_ros_msgs-msg:<ChemicalParticleConcentration> is deprecated: use uuv_sensor_plugins_ros_msgs-msg:ChemicalParticleConcentration instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ChemicalParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:header-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ChemicalParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:position-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <ChemicalParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:latitude-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <ChemicalParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:longitude-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <ChemicalParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:depth-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:depth instead.")
  (depth m))

(cl:ensure-generic-function 'concentration-val :lambda-list '(m))
(cl:defmethod concentration-val ((m <ChemicalParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:concentration-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:concentration instead.")
  (concentration m))

(cl:ensure-generic-function 'is_measuring-val :lambda-list '(m))
(cl:defmethod is_measuring-val ((m <ChemicalParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:is_measuring-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:is_measuring instead.")
  (is_measuring m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChemicalParticleConcentration>) ostream)
  "Serializes a message object of type '<ChemicalParticleConcentration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'concentration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_measuring) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChemicalParticleConcentration>) istream)
  "Deserializes a message object of type '<ChemicalParticleConcentration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'concentration) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'is_measuring) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChemicalParticleConcentration>)))
  "Returns string type for a message object of type '<ChemicalParticleConcentration>"
  "uuv_sensor_plugins_ros_msgs/ChemicalParticleConcentration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChemicalParticleConcentration)))
  "Returns string type for a message object of type 'ChemicalParticleConcentration"
  "uuv_sensor_plugins_ros_msgs/ChemicalParticleConcentration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChemicalParticleConcentration>)))
  "Returns md5sum for a message object of type '<ChemicalParticleConcentration>"
  "f17ac73d7dd3916a4fd6f721f0ccbe37")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChemicalParticleConcentration)))
  "Returns md5sum for a message object of type 'ChemicalParticleConcentration"
  "f17ac73d7dd3916a4fd6f721f0ccbe37")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChemicalParticleConcentration>)))
  "Returns full string definition for message of type '<ChemicalParticleConcentration>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%Header header~%# Sensor link position~%geometry_msgs/Vector3 position~%# Latitude in degrees (WGS84 coordinates)~%float64 latitude~%# Longiture in degrees (WGS84 coordinates)~%float64 longitude~%# Depth in meters (WGS84 coordinates)~%float64 depth~%# Particle concentration~%float64 concentration~%# Status, false if no plume particles are received~%bool is_measuring~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChemicalParticleConcentration)))
  "Returns full string definition for message of type 'ChemicalParticleConcentration"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%Header header~%# Sensor link position~%geometry_msgs/Vector3 position~%# Latitude in degrees (WGS84 coordinates)~%float64 latitude~%# Longiture in degrees (WGS84 coordinates)~%float64 longitude~%# Depth in meters (WGS84 coordinates)~%float64 depth~%# Particle concentration~%float64 concentration~%# Status, false if no plume particles are received~%bool is_measuring~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChemicalParticleConcentration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     8
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChemicalParticleConcentration>))
  "Converts a ROS message object to a list"
  (cl:list 'ChemicalParticleConcentration
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':depth (depth msg))
    (cl:cons ':concentration (concentration msg))
    (cl:cons ':is_measuring (is_measuring msg))
))
