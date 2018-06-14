; Auto-generated. Do not edit!


(cl:in-package uuv_auv_control_allocator-msg)


;//! \htmlinclude AUVCommand.msg.html

(cl:defclass <AUVCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (surge_speed
    :reader surge_speed
    :initarg :surge_speed
    :type cl:float
    :initform 0.0)
   (command
    :reader command
    :initarg :command
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench)))
)

(cl:defclass AUVCommand (<AUVCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AUVCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AUVCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_auv_control_allocator-msg:<AUVCommand> is deprecated: use uuv_auv_control_allocator-msg:AUVCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AUVCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_auv_control_allocator-msg:header-val is deprecated.  Use uuv_auv_control_allocator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'surge_speed-val :lambda-list '(m))
(cl:defmethod surge_speed-val ((m <AUVCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_auv_control_allocator-msg:surge_speed-val is deprecated.  Use uuv_auv_control_allocator-msg:surge_speed instead.")
  (surge_speed m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <AUVCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_auv_control_allocator-msg:command-val is deprecated.  Use uuv_auv_control_allocator-msg:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AUVCommand>) ostream)
  "Serializes a message object of type '<AUVCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'surge_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AUVCommand>) istream)
  "Deserializes a message object of type '<AUVCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'surge_speed) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AUVCommand>)))
  "Returns string type for a message object of type '<AUVCommand>"
  "uuv_auv_control_allocator/AUVCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AUVCommand)))
  "Returns string type for a message object of type 'AUVCommand"
  "uuv_auv_control_allocator/AUVCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AUVCommand>)))
  "Returns md5sum for a message object of type '<AUVCommand>"
  "9d7c962f08b7f118399273df23351e7c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AUVCommand)))
  "Returns md5sum for a message object of type 'AUVCommand"
  "9d7c962f08b7f118399273df23351e7c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AUVCommand>)))
  "Returns full string definition for message of type '<AUVCommand>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%float64 surge_speed~%geometry_msgs/Wrench command~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AUVCommand)))
  "Returns full string definition for message of type 'AUVCommand"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%float64 surge_speed~%geometry_msgs/Wrench command~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AUVCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AUVCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'AUVCommand
    (cl:cons ':header (header msg))
    (cl:cons ':surge_speed (surge_speed msg))
    (cl:cons ':command (command msg))
))
