; Auto-generated. Do not edit!


(cl:in-package uuv_gazebo_ros_plugins_msgs-srv)


;//! \htmlinclude GetModelProperties-request.msg.html

(cl:defclass <GetModelProperties-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetModelProperties-request (<GetModelProperties-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModelProperties-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModelProperties-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<GetModelProperties-request> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:GetModelProperties-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModelProperties-request>) ostream)
  "Serializes a message object of type '<GetModelProperties-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModelProperties-request>) istream)
  "Deserializes a message object of type '<GetModelProperties-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModelProperties-request>)))
  "Returns string type for a service object of type '<GetModelProperties-request>"
  "uuv_gazebo_ros_plugins_msgs/GetModelPropertiesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModelProperties-request)))
  "Returns string type for a service object of type 'GetModelProperties-request"
  "uuv_gazebo_ros_plugins_msgs/GetModelPropertiesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModelProperties-request>)))
  "Returns md5sum for a message object of type '<GetModelProperties-request>"
  "222d64ab6fa46c24f1abd065170ebe7a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModelProperties-request)))
  "Returns md5sum for a message object of type 'GetModelProperties-request"
  "222d64ab6fa46c24f1abd065170ebe7a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModelProperties-request>)))
  "Returns full string definition for message of type '<GetModelProperties-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModelProperties-request)))
  "Returns full string definition for message of type 'GetModelProperties-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModelProperties-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModelProperties-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModelProperties-request
))
;//! \htmlinclude GetModelProperties-response.msg.html

(cl:defclass <GetModelProperties-response> (roslisp-msg-protocol:ros-message)
  ((link_names
    :reader link_names
    :initarg :link_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (models
    :reader models
    :initarg :models
    :type (cl:vector uuv_gazebo_ros_plugins_msgs-msg:UnderwaterObjectModel)
   :initform (cl:make-array 0 :element-type 'uuv_gazebo_ros_plugins_msgs-msg:UnderwaterObjectModel :initial-element (cl:make-instance 'uuv_gazebo_ros_plugins_msgs-msg:UnderwaterObjectModel))))
)

(cl:defclass GetModelProperties-response (<GetModelProperties-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModelProperties-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModelProperties-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<GetModelProperties-response> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:GetModelProperties-response instead.")))

(cl:ensure-generic-function 'link_names-val :lambda-list '(m))
(cl:defmethod link_names-val ((m <GetModelProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:link_names-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:link_names instead.")
  (link_names m))

(cl:ensure-generic-function 'models-val :lambda-list '(m))
(cl:defmethod models-val ((m <GetModelProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:models-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:models instead.")
  (models m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModelProperties-response>) ostream)
  "Serializes a message object of type '<GetModelProperties-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'link_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'link_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'models))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'models))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModelProperties-response>) istream)
  "Deserializes a message object of type '<GetModelProperties-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'link_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'link_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'models) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'models)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'uuv_gazebo_ros_plugins_msgs-msg:UnderwaterObjectModel))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModelProperties-response>)))
  "Returns string type for a service object of type '<GetModelProperties-response>"
  "uuv_gazebo_ros_plugins_msgs/GetModelPropertiesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModelProperties-response)))
  "Returns string type for a service object of type 'GetModelProperties-response"
  "uuv_gazebo_ros_plugins_msgs/GetModelPropertiesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModelProperties-response>)))
  "Returns md5sum for a message object of type '<GetModelProperties-response>"
  "222d64ab6fa46c24f1abd065170ebe7a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModelProperties-response)))
  "Returns md5sum for a message object of type 'GetModelProperties-response"
  "222d64ab6fa46c24f1abd065170ebe7a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModelProperties-response>)))
  "Returns full string definition for message of type '<GetModelProperties-response>"
  (cl:format cl:nil "string[]  link_names~%uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel[] models~%~%~%================================================================================~%MSG: uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%float64[] added_mass~%float64[] linear_damping~%float64[] linear_damping_forward_speed~%float64[] quadratic_damping~%float64 volume~%float64 bbox_height~%float64 bbox_length~%float64 bbox_width~%float64 fluid_density~%geometry_msgs/Vector3 cob~%bool neutrally_buoyant~%geometry_msgs/Inertia inertia~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Inertia~%# Mass [kg]~%float64 m~%~%# Center of mass [m]~%geometry_msgs/Vector3 com~%~%# Inertia Tensor [kg-m^2]~%#     | ixx ixy ixz |~%# I = | ixy iyy iyz |~%#     | ixz iyz izz |~%float64 ixx~%float64 ixy~%float64 ixz~%float64 iyy~%float64 iyz~%float64 izz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModelProperties-response)))
  "Returns full string definition for message of type 'GetModelProperties-response"
  (cl:format cl:nil "string[]  link_names~%uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel[] models~%~%~%================================================================================~%MSG: uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%float64[] added_mass~%float64[] linear_damping~%float64[] linear_damping_forward_speed~%float64[] quadratic_damping~%float64 volume~%float64 bbox_height~%float64 bbox_length~%float64 bbox_width~%float64 fluid_density~%geometry_msgs/Vector3 cob~%bool neutrally_buoyant~%geometry_msgs/Inertia inertia~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Inertia~%# Mass [kg]~%float64 m~%~%# Center of mass [m]~%geometry_msgs/Vector3 com~%~%# Inertia Tensor [kg-m^2]~%#     | ixx ixy ixz |~%# I = | ixy iyy iyz |~%#     | ixz iyz izz |~%float64 ixx~%float64 ixy~%float64 ixz~%float64 iyy~%float64 iyz~%float64 izz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModelProperties-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'link_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'models) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModelProperties-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModelProperties-response
    (cl:cons ':link_names (link_names msg))
    (cl:cons ':models (models msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetModelProperties)))
  'GetModelProperties-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetModelProperties)))
  'GetModelProperties-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModelProperties)))
  "Returns string type for a service object of type '<GetModelProperties>"
  "uuv_gazebo_ros_plugins_msgs/GetModelProperties")