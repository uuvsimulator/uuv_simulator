; Auto-generated. Do not edit!


(cl:in-package uuv_sensor_plugins_ros_msgs-msg)


;//! \htmlinclude DVL.msg.html

(cl:defclass <DVL> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity_covariance
    :reader velocity_covariance
    :initarg :velocity_covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0)
   (beams
    :reader beams
    :initarg :beams
    :type (cl:vector uuv_sensor_plugins_ros_msgs-msg:DVLBeam)
   :initform (cl:make-array 0 :element-type 'uuv_sensor_plugins_ros_msgs-msg:DVLBeam :initial-element (cl:make-instance 'uuv_sensor_plugins_ros_msgs-msg:DVLBeam))))
)

(cl:defclass DVL (<DVL>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DVL>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DVL)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_sensor_plugins_ros_msgs-msg:<DVL> is deprecated: use uuv_sensor_plugins_ros_msgs-msg:DVL instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DVL>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:header-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <DVL>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:velocity-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'velocity_covariance-val :lambda-list '(m))
(cl:defmethod velocity_covariance-val ((m <DVL>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:velocity_covariance-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:velocity_covariance instead.")
  (velocity_covariance m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <DVL>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:altitude-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:altitude instead.")
  (altitude m))

(cl:ensure-generic-function 'beams-val :lambda-list '(m))
(cl:defmethod beams-val ((m <DVL>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-msg:beams-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-msg:beams instead.")
  (beams m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DVL>) ostream)
  "Serializes a message object of type '<DVL>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'velocity_covariance))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'beams))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'beams))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DVL>) istream)
  "Deserializes a message object of type '<DVL>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (cl:setf (cl:slot-value msg 'velocity_covariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'velocity_covariance)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'beams) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'beams)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'uuv_sensor_plugins_ros_msgs-msg:DVLBeam))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DVL>)))
  "Returns string type for a message object of type '<DVL>"
  "uuv_sensor_plugins_ros_msgs/DVL")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DVL)))
  "Returns string type for a message object of type 'DVL"
  "uuv_sensor_plugins_ros_msgs/DVL")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DVL>)))
  "Returns md5sum for a message object of type '<DVL>"
  "02bba6182b6f271447d7f88473256572")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DVL)))
  "Returns md5sum for a message object of type 'DVL"
  "02bba6182b6f271447d7f88473256572")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DVL>)))
  "Returns full string definition for message of type '<DVL>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This is a message to hold data from a DVL sensor (Doppler Velocity Log).~%#~%# Distances are in [m], velocities in [m/s]~%#~%# If the covariance is known, it should be filled.~%# If it is unknown, it should be set to all zeros.~%# If a measurement was invalid, its covariance should be set to -1 so it can be~%# disregarded.~%#~%# DVLBeams are optional. If they are set they contain individual ranges and 1D~%# doppler velocity estimates orthogonal to the ray.~%~%Header header~%# Measured velocity [m/s]~%geometry_msgs/Vector3 velocity~%# Row major, xyz axes~%float64[9] velocity_covariance~%# Altitude of the vehicle~%float64 altitude~%DVLBeam[] beams~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: uuv_sensor_plugins_ros_msgs/DVLBeam~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# measured range [m] or < 0 if invalid~%float64 range~%float64 range_covariance~%~%float64 velocity # measured velocity [m/s] of corr. beam~%float64 velocity_covariance~%~%# Beam link pose wrt DVL link frame~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DVL)))
  "Returns full string definition for message of type 'DVL"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This is a message to hold data from a DVL sensor (Doppler Velocity Log).~%#~%# Distances are in [m], velocities in [m/s]~%#~%# If the covariance is known, it should be filled.~%# If it is unknown, it should be set to all zeros.~%# If a measurement was invalid, its covariance should be set to -1 so it can be~%# disregarded.~%#~%# DVLBeams are optional. If they are set they contain individual ranges and 1D~%# doppler velocity estimates orthogonal to the ray.~%~%Header header~%# Measured velocity [m/s]~%geometry_msgs/Vector3 velocity~%# Row major, xyz axes~%float64[9] velocity_covariance~%# Altitude of the vehicle~%float64 altitude~%DVLBeam[] beams~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: uuv_sensor_plugins_ros_msgs/DVLBeam~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# measured range [m] or < 0 if invalid~%float64 range~%float64 range_covariance~%~%float64 velocity # measured velocity [m/s] of corr. beam~%float64 velocity_covariance~%~%# Beam link pose wrt DVL link frame~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DVL>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'velocity_covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'beams) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DVL>))
  "Converts a ROS message object to a list"
  (cl:list 'DVL
    (cl:cons ':header (header msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':velocity_covariance (velocity_covariance msg))
    (cl:cons ':altitude (altitude msg))
    (cl:cons ':beams (beams msg))
))
