; Auto-generated. Do not edit!


(cl:in-package uuv_gazebo_ros_plugins_msgs-srv)


;//! \htmlinclude SetThrusterState-request.msg.html

(cl:defclass <SetThrusterState-request> (roslisp-msg-protocol:ros-message)
  ((on
    :reader on
    :initarg :on
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetThrusterState-request (<SetThrusterState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetThrusterState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetThrusterState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<SetThrusterState-request> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:SetThrusterState-request instead.")))

(cl:ensure-generic-function 'on-val :lambda-list '(m))
(cl:defmethod on-val ((m <SetThrusterState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:on-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:on instead.")
  (on m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetThrusterState-request>) ostream)
  "Serializes a message object of type '<SetThrusterState-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'on) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetThrusterState-request>) istream)
  "Deserializes a message object of type '<SetThrusterState-request>"
    (cl:setf (cl:slot-value msg 'on) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetThrusterState-request>)))
  "Returns string type for a service object of type '<SetThrusterState-request>"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThrusterState-request)))
  "Returns string type for a service object of type 'SetThrusterState-request"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetThrusterState-request>)))
  "Returns md5sum for a message object of type '<SetThrusterState-request>"
  "7abefa06f2be45ab014b8164e7591960")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetThrusterState-request)))
  "Returns md5sum for a message object of type 'SetThrusterState-request"
  "7abefa06f2be45ab014b8164e7591960")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetThrusterState-request>)))
  "Returns full string definition for message of type '<SetThrusterState-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool on~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetThrusterState-request)))
  "Returns full string definition for message of type 'SetThrusterState-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool on~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetThrusterState-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetThrusterState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetThrusterState-request
    (cl:cons ':on (on msg))
))
;//! \htmlinclude SetThrusterState-response.msg.html

(cl:defclass <SetThrusterState-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetThrusterState-response (<SetThrusterState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetThrusterState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetThrusterState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<SetThrusterState-response> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:SetThrusterState-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetThrusterState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:success-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetThrusterState-response>) ostream)
  "Serializes a message object of type '<SetThrusterState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetThrusterState-response>) istream)
  "Deserializes a message object of type '<SetThrusterState-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetThrusterState-response>)))
  "Returns string type for a service object of type '<SetThrusterState-response>"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThrusterState-response)))
  "Returns string type for a service object of type 'SetThrusterState-response"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetThrusterState-response>)))
  "Returns md5sum for a message object of type '<SetThrusterState-response>"
  "7abefa06f2be45ab014b8164e7591960")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetThrusterState-response)))
  "Returns md5sum for a message object of type 'SetThrusterState-response"
  "7abefa06f2be45ab014b8164e7591960")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetThrusterState-response>)))
  "Returns full string definition for message of type '<SetThrusterState-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetThrusterState-response)))
  "Returns full string definition for message of type 'SetThrusterState-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetThrusterState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetThrusterState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetThrusterState-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetThrusterState)))
  'SetThrusterState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetThrusterState)))
  'SetThrusterState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThrusterState)))
  "Returns string type for a service object of type '<SetThrusterState>"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterState")