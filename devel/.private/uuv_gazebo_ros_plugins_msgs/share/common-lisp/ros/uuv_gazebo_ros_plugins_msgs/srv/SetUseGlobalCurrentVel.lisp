; Auto-generated. Do not edit!


(cl:in-package uuv_gazebo_ros_plugins_msgs-srv)


;//! \htmlinclude SetUseGlobalCurrentVel-request.msg.html

(cl:defclass <SetUseGlobalCurrentVel-request> (roslisp-msg-protocol:ros-message)
  ((use_global
    :reader use_global
    :initarg :use_global
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetUseGlobalCurrentVel-request (<SetUseGlobalCurrentVel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUseGlobalCurrentVel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUseGlobalCurrentVel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<SetUseGlobalCurrentVel-request> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:SetUseGlobalCurrentVel-request instead.")))

(cl:ensure-generic-function 'use_global-val :lambda-list '(m))
(cl:defmethod use_global-val ((m <SetUseGlobalCurrentVel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:use_global-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:use_global instead.")
  (use_global m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUseGlobalCurrentVel-request>) ostream)
  "Serializes a message object of type '<SetUseGlobalCurrentVel-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_global) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUseGlobalCurrentVel-request>) istream)
  "Deserializes a message object of type '<SetUseGlobalCurrentVel-request>"
    (cl:setf (cl:slot-value msg 'use_global) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUseGlobalCurrentVel-request>)))
  "Returns string type for a service object of type '<SetUseGlobalCurrentVel-request>"
  "uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUseGlobalCurrentVel-request)))
  "Returns string type for a service object of type 'SetUseGlobalCurrentVel-request"
  "uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUseGlobalCurrentVel-request>)))
  "Returns md5sum for a message object of type '<SetUseGlobalCurrentVel-request>"
  "02d40f951486d0b4bee34e7b1c66f745")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUseGlobalCurrentVel-request)))
  "Returns md5sum for a message object of type 'SetUseGlobalCurrentVel-request"
  "02d40f951486d0b4bee34e7b1c66f745")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUseGlobalCurrentVel-request>)))
  "Returns full string definition for message of type '<SetUseGlobalCurrentVel-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool use_global~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUseGlobalCurrentVel-request)))
  "Returns full string definition for message of type 'SetUseGlobalCurrentVel-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool use_global~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUseGlobalCurrentVel-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUseGlobalCurrentVel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUseGlobalCurrentVel-request
    (cl:cons ':use_global (use_global msg))
))
;//! \htmlinclude SetUseGlobalCurrentVel-response.msg.html

(cl:defclass <SetUseGlobalCurrentVel-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetUseGlobalCurrentVel-response (<SetUseGlobalCurrentVel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUseGlobalCurrentVel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUseGlobalCurrentVel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<SetUseGlobalCurrentVel-response> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:SetUseGlobalCurrentVel-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetUseGlobalCurrentVel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:success-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUseGlobalCurrentVel-response>) ostream)
  "Serializes a message object of type '<SetUseGlobalCurrentVel-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUseGlobalCurrentVel-response>) istream)
  "Deserializes a message object of type '<SetUseGlobalCurrentVel-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUseGlobalCurrentVel-response>)))
  "Returns string type for a service object of type '<SetUseGlobalCurrentVel-response>"
  "uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUseGlobalCurrentVel-response)))
  "Returns string type for a service object of type 'SetUseGlobalCurrentVel-response"
  "uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUseGlobalCurrentVel-response>)))
  "Returns md5sum for a message object of type '<SetUseGlobalCurrentVel-response>"
  "02d40f951486d0b4bee34e7b1c66f745")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUseGlobalCurrentVel-response)))
  "Returns md5sum for a message object of type 'SetUseGlobalCurrentVel-response"
  "02d40f951486d0b4bee34e7b1c66f745")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUseGlobalCurrentVel-response>)))
  "Returns full string definition for message of type '<SetUseGlobalCurrentVel-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUseGlobalCurrentVel-response)))
  "Returns full string definition for message of type 'SetUseGlobalCurrentVel-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUseGlobalCurrentVel-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUseGlobalCurrentVel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUseGlobalCurrentVel-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetUseGlobalCurrentVel)))
  'SetUseGlobalCurrentVel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetUseGlobalCurrentVel)))
  'SetUseGlobalCurrentVel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUseGlobalCurrentVel)))
  "Returns string type for a service object of type '<SetUseGlobalCurrentVel>"
  "uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVel")