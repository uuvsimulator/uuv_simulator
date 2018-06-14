; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude GoTo-request.msg.html

(cl:defclass <GoTo-request> (roslisp-msg-protocol:ros-message)
  ((waypoint
    :reader waypoint
    :initarg :waypoint
    :type uuv_control_msgs-msg:Waypoint
    :initform (cl:make-instance 'uuv_control_msgs-msg:Waypoint))
   (max_forward_speed
    :reader max_forward_speed
    :initarg :max_forward_speed
    :type cl:float
    :initform 0.0)
   (interpolator
    :reader interpolator
    :initarg :interpolator
    :type cl:string
    :initform ""))
)

(cl:defclass GoTo-request (<GoTo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoTo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoTo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<GoTo-request> is deprecated: use uuv_control_msgs-srv:GoTo-request instead.")))

(cl:ensure-generic-function 'waypoint-val :lambda-list '(m))
(cl:defmethod waypoint-val ((m <GoTo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:waypoint-val is deprecated.  Use uuv_control_msgs-srv:waypoint instead.")
  (waypoint m))

(cl:ensure-generic-function 'max_forward_speed-val :lambda-list '(m))
(cl:defmethod max_forward_speed-val ((m <GoTo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:max_forward_speed-val is deprecated.  Use uuv_control_msgs-srv:max_forward_speed instead.")
  (max_forward_speed m))

(cl:ensure-generic-function 'interpolator-val :lambda-list '(m))
(cl:defmethod interpolator-val ((m <GoTo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:interpolator-val is deprecated.  Use uuv_control_msgs-srv:interpolator instead.")
  (interpolator m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GoTo-request>)))
    "Constants for message type '<GoTo-request>"
  '((:LIPB . 'lipb')
    (:CUBIC . 'cubic')
    (:DUBINS . 'dubins')
    (:LINEAR . 'linear'))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GoTo-request)))
    "Constants for message type 'GoTo-request"
  '((:LIPB . 'lipb')
    (:CUBIC . 'cubic')
    (:DUBINS . 'dubins')
    (:LINEAR . 'linear'))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoTo-request>) ostream)
  "Serializes a message object of type '<GoTo-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'waypoint) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_forward_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'interpolator))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'interpolator))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoTo-request>) istream)
  "Deserializes a message object of type '<GoTo-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'waypoint) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_forward_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'interpolator) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'interpolator) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoTo-request>)))
  "Returns string type for a service object of type '<GoTo-request>"
  "uuv_control_msgs/GoToRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoTo-request)))
  "Returns string type for a service object of type 'GoTo-request"
  "uuv_control_msgs/GoToRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoTo-request>)))
  "Returns md5sum for a message object of type '<GoTo-request>"
  "408446fa9ec1d90b38a7053e3dd0ad47")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoTo-request)))
  "Returns md5sum for a message object of type 'GoTo-request"
  "408446fa9ec1d90b38a7053e3dd0ad47")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoTo-request>)))
  "Returns full string definition for message of type '<GoTo-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string LIPB='lipb'~%string CUBIC='cubic'~%string DUBINS='dubins'~%string LINEAR='linear'~%~%uuv_control_msgs/Waypoint waypoint~%float64 max_forward_speed~%string interpolator~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoTo-request)))
  "Returns full string definition for message of type 'GoTo-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string LIPB='lipb'~%string CUBIC='cubic'~%string DUBINS='dubins'~%string LINEAR='linear'~%~%uuv_control_msgs/Waypoint waypoint~%float64 max_forward_speed~%string interpolator~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoTo-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'waypoint))
     8
     4 (cl:length (cl:slot-value msg 'interpolator))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoTo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GoTo-request
    (cl:cons ':waypoint (waypoint msg))
    (cl:cons ':max_forward_speed (max_forward_speed msg))
    (cl:cons ':interpolator (interpolator msg))
))
;//! \htmlinclude GoTo-response.msg.html

(cl:defclass <GoTo-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GoTo-response (<GoTo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoTo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoTo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<GoTo-response> is deprecated: use uuv_control_msgs-srv:GoTo-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GoTo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoTo-response>) ostream)
  "Serializes a message object of type '<GoTo-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoTo-response>) istream)
  "Deserializes a message object of type '<GoTo-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoTo-response>)))
  "Returns string type for a service object of type '<GoTo-response>"
  "uuv_control_msgs/GoToResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoTo-response)))
  "Returns string type for a service object of type 'GoTo-response"
  "uuv_control_msgs/GoToResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoTo-response>)))
  "Returns md5sum for a message object of type '<GoTo-response>"
  "408446fa9ec1d90b38a7053e3dd0ad47")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoTo-response)))
  "Returns md5sum for a message object of type 'GoTo-response"
  "408446fa9ec1d90b38a7053e3dd0ad47")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoTo-response>)))
  "Returns full string definition for message of type '<GoTo-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoTo-response)))
  "Returns full string definition for message of type 'GoTo-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoTo-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoTo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoTo-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoTo)))
  'GoTo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoTo)))
  'GoTo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoTo)))
  "Returns string type for a service object of type '<GoTo>"
  "uuv_control_msgs/GoTo")