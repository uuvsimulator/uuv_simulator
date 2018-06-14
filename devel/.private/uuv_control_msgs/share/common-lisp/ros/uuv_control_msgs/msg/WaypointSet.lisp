; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-msg)


;//! \htmlinclude WaypointSet.msg.html

(cl:defclass <WaypointSet> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (start_time
    :reader start_time
    :initarg :start_time
    :type std_msgs-msg:Time
    :initform (cl:make-instance 'std_msgs-msg:Time))
   (waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector uuv_control_msgs-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'uuv_control_msgs-msg:Waypoint :initial-element (cl:make-instance 'uuv_control_msgs-msg:Waypoint))))
)

(cl:defclass WaypointSet (<WaypointSet>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointSet>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointSet)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-msg:<WaypointSet> is deprecated: use uuv_control_msgs-msg:WaypointSet instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WaypointSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-msg:header-val is deprecated.  Use uuv_control_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <WaypointSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-msg:start_time-val is deprecated.  Use uuv_control_msgs-msg:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <WaypointSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-msg:waypoints-val is deprecated.  Use uuv_control_msgs-msg:waypoints instead.")
  (waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointSet>) ostream)
  "Serializes a message object of type '<WaypointSet>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_time) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointSet>) istream)
  "Deserializes a message object of type '<WaypointSet>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_time) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointSet>)))
  "Returns string type for a message object of type '<WaypointSet>"
  "uuv_control_msgs/WaypointSet")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointSet)))
  "Returns string type for a message object of type 'WaypointSet"
  "uuv_control_msgs/WaypointSet")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointSet>)))
  "Returns md5sum for a message object of type '<WaypointSet>"
  "a946aaa8d17e35fe5a52e4f7ac482b5d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointSet)))
  "Returns md5sum for a message object of type 'WaypointSet"
  "a946aaa8d17e35fe5a52e4f7ac482b5d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointSet>)))
  "Returns full string definition for message of type '<WaypointSet>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%std_msgs/Time start_time~%uuv_control_msgs/Waypoint[] waypoints~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointSet)))
  "Returns full string definition for message of type 'WaypointSet"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%std_msgs/Time start_time~%uuv_control_msgs/Waypoint[] waypoints~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: uuv_control_msgs/Waypoint~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%std_msgs/Header header~%geometry_msgs/Point point~%float64 max_forward_speed~%float64 heading_offset~%bool use_fixed_heading~%float64 radius_of_acceptance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointSet>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_time))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointSet>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointSet
    (cl:cons ':header (header msg))
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':waypoints (waypoints msg))
))
