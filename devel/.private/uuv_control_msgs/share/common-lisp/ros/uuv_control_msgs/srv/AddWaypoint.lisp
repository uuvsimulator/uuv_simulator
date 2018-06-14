; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude AddWaypoint-request.msg.html

(cl:defclass <AddWaypoint-request> (roslisp-msg-protocol:ros-message)
  ((waypoint
    :reader waypoint
    :initarg :waypoint
    :type uuv_control_msgs-msg:Waypoint
    :initform (cl:make-instance 'uuv_control_msgs-msg:Waypoint)))
)

(cl:defclass AddWaypoint-request (<AddWaypoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddWaypoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddWaypoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<AddWaypoint-request> is deprecated: use uuv_control_msgs-srv:AddWaypoint-request instead.")))

(cl:ensure-generic-function 'waypoint-val :lambda-list '(m))
(cl:defmethod waypoint-val ((m <AddWaypoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:waypoint-val is deprecated.  Use uuv_control_msgs-srv:waypoint instead.")
  (waypoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddWaypoint-request>) ostream)
  "Serializes a message object of type '<AddWaypoint-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'waypoint) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddWaypoint-request>) istream)
  "Deserializes a message object of type '<AddWaypoint-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'waypoint) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddWaypoint-request>)))
  "Returns string type for a service object of type '<AddWaypoint-request>"
  "uuv_control_msgs/AddWaypointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddWaypoint-request)))
  "Returns string type for a service object of type 'AddWaypoint-request"
  "uuv_control_msgs/AddWaypointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddWaypoint-request>)))
  "Returns md5sum for a message object of type '<AddWaypoint-request>"
  "e853788769392728a6445812f447d75e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddWaypoint-request)))
  "Returns md5sum for a message object of type 'AddWaypoint-request"
  "e853788769392728a6445812f447d75e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddWaypoint-request>)))
  "Returns full string definition for message of type '<AddWaypoint-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uuv_control_msgs/Waypoint waypoint~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddWaypoint-request)))
  "Returns full string definition for message of type 'AddWaypoint-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uuv_control_msgs/Waypoint waypoint~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddWaypoint-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'waypoint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddWaypoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddWaypoint-request
    (cl:cons ':waypoint (waypoint msg))
))
;//! \htmlinclude AddWaypoint-response.msg.html

(cl:defclass <AddWaypoint-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector uuv_control_msgs-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'uuv_control_msgs-msg:Waypoint :initial-element (cl:make-instance 'uuv_control_msgs-msg:Waypoint))))
)

(cl:defclass AddWaypoint-response (<AddWaypoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddWaypoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddWaypoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<AddWaypoint-response> is deprecated: use uuv_control_msgs-srv:AddWaypoint-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddWaypoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <AddWaypoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:waypoints-val is deprecated.  Use uuv_control_msgs-srv:waypoints instead.")
  (waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddWaypoint-response>) ostream)
  "Serializes a message object of type '<AddWaypoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddWaypoint-response>) istream)
  "Deserializes a message object of type '<AddWaypoint-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddWaypoint-response>)))
  "Returns string type for a service object of type '<AddWaypoint-response>"
  "uuv_control_msgs/AddWaypointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddWaypoint-response)))
  "Returns string type for a service object of type 'AddWaypoint-response"
  "uuv_control_msgs/AddWaypointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddWaypoint-response>)))
  "Returns md5sum for a message object of type '<AddWaypoint-response>"
  "e853788769392728a6445812f447d75e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddWaypoint-response)))
  "Returns md5sum for a message object of type 'AddWaypoint-response"
  "e853788769392728a6445812f447d75e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddWaypoint-response>)))
  "Returns full string definition for message of type '<AddWaypoint-response>"
  (cl:format cl:nil "bool success~%uuv_control_msgs/Waypoint[] waypoints~%~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddWaypoint-response)))
  "Returns full string definition for message of type 'AddWaypoint-response"
  (cl:format cl:nil "bool success~%uuv_control_msgs/Waypoint[] waypoints~%~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddWaypoint-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddWaypoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddWaypoint-response
    (cl:cons ':success (success msg))
    (cl:cons ':waypoints (waypoints msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddWaypoint)))
  'AddWaypoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddWaypoint)))
  'AddWaypoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddWaypoint)))
  "Returns string type for a service object of type '<AddWaypoint>"
  "uuv_control_msgs/AddWaypoint")