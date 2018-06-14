; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude InitWaypointSet-request.msg.html

(cl:defclass <InitWaypointSet-request> (roslisp-msg-protocol:ros-message)
  ((start_time
    :reader start_time
    :initarg :start_time
    :type std_msgs-msg:Time
    :initform (cl:make-instance 'std_msgs-msg:Time))
   (start_now
    :reader start_now
    :initarg :start_now
    :type cl:boolean
    :initform cl:nil)
   (waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector uuv_control_msgs-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'uuv_control_msgs-msg:Waypoint :initial-element (cl:make-instance 'uuv_control_msgs-msg:Waypoint)))
   (max_forward_speed
    :reader max_forward_speed
    :initarg :max_forward_speed
    :type cl:float
    :initform 0.0)
   (heading_offset
    :reader heading_offset
    :initarg :heading_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass InitWaypointSet-request (<InitWaypointSet-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitWaypointSet-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitWaypointSet-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<InitWaypointSet-request> is deprecated: use uuv_control_msgs-srv:InitWaypointSet-request instead.")))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <InitWaypointSet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:start_time-val is deprecated.  Use uuv_control_msgs-srv:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'start_now-val :lambda-list '(m))
(cl:defmethod start_now-val ((m <InitWaypointSet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:start_now-val is deprecated.  Use uuv_control_msgs-srv:start_now instead.")
  (start_now m))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <InitWaypointSet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:waypoints-val is deprecated.  Use uuv_control_msgs-srv:waypoints instead.")
  (waypoints m))

(cl:ensure-generic-function 'max_forward_speed-val :lambda-list '(m))
(cl:defmethod max_forward_speed-val ((m <InitWaypointSet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:max_forward_speed-val is deprecated.  Use uuv_control_msgs-srv:max_forward_speed instead.")
  (max_forward_speed m))

(cl:ensure-generic-function 'heading_offset-val :lambda-list '(m))
(cl:defmethod heading_offset-val ((m <InitWaypointSet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:heading_offset-val is deprecated.  Use uuv_control_msgs-srv:heading_offset instead.")
  (heading_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitWaypointSet-request>) ostream)
  "Serializes a message object of type '<InitWaypointSet-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_time) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start_now) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_forward_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitWaypointSet-request>) istream)
  "Deserializes a message object of type '<InitWaypointSet-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_time) istream)
    (cl:setf (cl:slot-value msg 'start_now) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'uuv_control_msgs-msg:Waypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_offset) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitWaypointSet-request>)))
  "Returns string type for a service object of type '<InitWaypointSet-request>"
  "uuv_control_msgs/InitWaypointSetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitWaypointSet-request)))
  "Returns string type for a service object of type 'InitWaypointSet-request"
  "uuv_control_msgs/InitWaypointSetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitWaypointSet-request>)))
  "Returns md5sum for a message object of type '<InitWaypointSet-request>"
  "a5fe64b7879e685af95ae8210e45b1c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitWaypointSet-request)))
  "Returns md5sum for a message object of type 'InitWaypointSet-request"
  "a5fe64b7879e685af95ae8210e45b1c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitWaypointSet-request>)))
  "Returns full string definition for message of type '<InitWaypointSet-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%std_msgs/Time start_time~%bool start_now~%uuv_control_msgs/Waypoint[] waypoints~%float64 max_forward_speed~%float64 heading_offset~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitWaypointSet-request)))
  "Returns full string definition for message of type 'InitWaypointSet-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%std_msgs/Time start_time~%bool start_now~%uuv_control_msgs/Waypoint[] waypoints~%float64 max_forward_speed~%float64 heading_offset~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitWaypointSet-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_time))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitWaypointSet-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InitWaypointSet-request
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':start_now (start_now msg))
    (cl:cons ':waypoints (waypoints msg))
    (cl:cons ':max_forward_speed (max_forward_speed msg))
    (cl:cons ':heading_offset (heading_offset msg))
))
;//! \htmlinclude InitWaypointSet-response.msg.html

(cl:defclass <InitWaypointSet-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass InitWaypointSet-response (<InitWaypointSet-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitWaypointSet-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitWaypointSet-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<InitWaypointSet-response> is deprecated: use uuv_control_msgs-srv:InitWaypointSet-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <InitWaypointSet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitWaypointSet-response>) ostream)
  "Serializes a message object of type '<InitWaypointSet-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitWaypointSet-response>) istream)
  "Deserializes a message object of type '<InitWaypointSet-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitWaypointSet-response>)))
  "Returns string type for a service object of type '<InitWaypointSet-response>"
  "uuv_control_msgs/InitWaypointSetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitWaypointSet-response)))
  "Returns string type for a service object of type 'InitWaypointSet-response"
  "uuv_control_msgs/InitWaypointSetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitWaypointSet-response>)))
  "Returns md5sum for a message object of type '<InitWaypointSet-response>"
  "a5fe64b7879e685af95ae8210e45b1c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitWaypointSet-response)))
  "Returns md5sum for a message object of type 'InitWaypointSet-response"
  "a5fe64b7879e685af95ae8210e45b1c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitWaypointSet-response>)))
  "Returns full string definition for message of type '<InitWaypointSet-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitWaypointSet-response)))
  "Returns full string definition for message of type 'InitWaypointSet-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitWaypointSet-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitWaypointSet-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InitWaypointSet-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InitWaypointSet)))
  'InitWaypointSet-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InitWaypointSet)))
  'InitWaypointSet-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitWaypointSet)))
  "Returns string type for a service object of type '<InitWaypointSet>"
  "uuv_control_msgs/InitWaypointSet")