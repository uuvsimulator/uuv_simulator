; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude GetWaypoints-request.msg.html

(cl:defclass <GetWaypoints-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetWaypoints-request (<GetWaypoints-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWaypoints-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWaypoints-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<GetWaypoints-request> is deprecated: use uuv_control_msgs-srv:GetWaypoints-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWaypoints-request>) ostream)
  "Serializes a message object of type '<GetWaypoints-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWaypoints-request>) istream)
  "Deserializes a message object of type '<GetWaypoints-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWaypoints-request>)))
  "Returns string type for a service object of type '<GetWaypoints-request>"
  "uuv_control_msgs/GetWaypointsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWaypoints-request)))
  "Returns string type for a service object of type 'GetWaypoints-request"
  "uuv_control_msgs/GetWaypointsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWaypoints-request>)))
  "Returns md5sum for a message object of type '<GetWaypoints-request>"
  "7256248537b12c726720758df9a72413")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWaypoints-request)))
  "Returns md5sum for a message object of type 'GetWaypoints-request"
  "7256248537b12c726720758df9a72413")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWaypoints-request>)))
  "Returns full string definition for message of type '<GetWaypoints-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWaypoints-request)))
  "Returns full string definition for message of type 'GetWaypoints-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWaypoints-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWaypoints-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWaypoints-request
))
;//! \htmlinclude GetWaypoints-response.msg.html

(cl:defclass <GetWaypoints-response> (roslisp-msg-protocol:ros-message)
  ((waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector uuv_control_msgs-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'uuv_control_msgs-msg:Waypoint :initial-element (cl:make-instance 'uuv_control_msgs-msg:Waypoint))))
)

(cl:defclass GetWaypoints-response (<GetWaypoints-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWaypoints-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWaypoints-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<GetWaypoints-response> is deprecated: use uuv_control_msgs-srv:GetWaypoints-response instead.")))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <GetWaypoints-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:waypoints-val is deprecated.  Use uuv_control_msgs-srv:waypoints instead.")
  (waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWaypoints-response>) ostream)
  "Serializes a message object of type '<GetWaypoints-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWaypoints-response>) istream)
  "Deserializes a message object of type '<GetWaypoints-response>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWaypoints-response>)))
  "Returns string type for a service object of type '<GetWaypoints-response>"
  "uuv_control_msgs/GetWaypointsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWaypoints-response)))
  "Returns string type for a service object of type 'GetWaypoints-response"
  "uuv_control_msgs/GetWaypointsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWaypoints-response>)))
  "Returns md5sum for a message object of type '<GetWaypoints-response>"
  "7256248537b12c726720758df9a72413")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWaypoints-response)))
  "Returns md5sum for a message object of type 'GetWaypoints-response"
  "7256248537b12c726720758df9a72413")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWaypoints-response>)))
  "Returns full string definition for message of type '<GetWaypoints-response>"
  (cl:format cl:nil "uuv_control_msgs/Waypoint[] waypoints~%~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWaypoints-response)))
  "Returns full string definition for message of type 'GetWaypoints-response"
  (cl:format cl:nil "uuv_control_msgs/Waypoint[] waypoints~%~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWaypoints-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWaypoints-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWaypoints-response
    (cl:cons ':waypoints (waypoints msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWaypoints)))
  'GetWaypoints-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWaypoints)))
  'GetWaypoints-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWaypoints)))
  "Returns string type for a service object of type '<GetWaypoints>"
  "uuv_control_msgs/GetWaypoints")