; Auto-generated. Do not edit!


(cl:in-package uuv_gazebo_ros_plugins_msgs-srv)


;//! \htmlinclude GetThrusterState-request.msg.html

(cl:defclass <GetThrusterState-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetThrusterState-request (<GetThrusterState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetThrusterState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetThrusterState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<GetThrusterState-request> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:GetThrusterState-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetThrusterState-request>) ostream)
  "Serializes a message object of type '<GetThrusterState-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetThrusterState-request>) istream)
  "Deserializes a message object of type '<GetThrusterState-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetThrusterState-request>)))
  "Returns string type for a service object of type '<GetThrusterState-request>"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterState-request)))
  "Returns string type for a service object of type 'GetThrusterState-request"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetThrusterState-request>)))
  "Returns md5sum for a message object of type '<GetThrusterState-request>"
  "e2fdda8431274beee70eebaa081c813e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetThrusterState-request)))
  "Returns md5sum for a message object of type 'GetThrusterState-request"
  "e2fdda8431274beee70eebaa081c813e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetThrusterState-request>)))
  "Returns full string definition for message of type '<GetThrusterState-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetThrusterState-request)))
  "Returns full string definition for message of type 'GetThrusterState-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetThrusterState-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetThrusterState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetThrusterState-request
))
;//! \htmlinclude GetThrusterState-response.msg.html

(cl:defclass <GetThrusterState-response> (roslisp-msg-protocol:ros-message)
  ((is_on
    :reader is_on
    :initarg :is_on
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetThrusterState-response (<GetThrusterState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetThrusterState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetThrusterState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<GetThrusterState-response> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:GetThrusterState-response instead.")))

(cl:ensure-generic-function 'is_on-val :lambda-list '(m))
(cl:defmethod is_on-val ((m <GetThrusterState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:is_on-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:is_on instead.")
  (is_on m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetThrusterState-response>) ostream)
  "Serializes a message object of type '<GetThrusterState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_on) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetThrusterState-response>) istream)
  "Deserializes a message object of type '<GetThrusterState-response>"
    (cl:setf (cl:slot-value msg 'is_on) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetThrusterState-response>)))
  "Returns string type for a service object of type '<GetThrusterState-response>"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterState-response)))
  "Returns string type for a service object of type 'GetThrusterState-response"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetThrusterState-response>)))
  "Returns md5sum for a message object of type '<GetThrusterState-response>"
  "e2fdda8431274beee70eebaa081c813e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetThrusterState-response)))
  "Returns md5sum for a message object of type 'GetThrusterState-response"
  "e2fdda8431274beee70eebaa081c813e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetThrusterState-response>)))
  "Returns full string definition for message of type '<GetThrusterState-response>"
  (cl:format cl:nil "bool is_on~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetThrusterState-response)))
  "Returns full string definition for message of type 'GetThrusterState-response"
  (cl:format cl:nil "bool is_on~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetThrusterState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetThrusterState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetThrusterState-response
    (cl:cons ':is_on (is_on msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetThrusterState)))
  'GetThrusterState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetThrusterState)))
  'GetThrusterState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterState)))
  "Returns string type for a service object of type '<GetThrusterState>"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterState")