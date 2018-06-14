; Auto-generated. Do not edit!


(cl:in-package uuv_gazebo_ros_plugins_msgs-srv)


;//! \htmlinclude GetThrusterEfficiency-request.msg.html

(cl:defclass <GetThrusterEfficiency-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetThrusterEfficiency-request (<GetThrusterEfficiency-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetThrusterEfficiency-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetThrusterEfficiency-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<GetThrusterEfficiency-request> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:GetThrusterEfficiency-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetThrusterEfficiency-request>) ostream)
  "Serializes a message object of type '<GetThrusterEfficiency-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetThrusterEfficiency-request>) istream)
  "Deserializes a message object of type '<GetThrusterEfficiency-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetThrusterEfficiency-request>)))
  "Returns string type for a service object of type '<GetThrusterEfficiency-request>"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiencyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterEfficiency-request)))
  "Returns string type for a service object of type 'GetThrusterEfficiency-request"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiencyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetThrusterEfficiency-request>)))
  "Returns md5sum for a message object of type '<GetThrusterEfficiency-request>"
  "b80ec71e671b93e4cc403df1ac4c8a86")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetThrusterEfficiency-request)))
  "Returns md5sum for a message object of type 'GetThrusterEfficiency-request"
  "b80ec71e671b93e4cc403df1ac4c8a86")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetThrusterEfficiency-request>)))
  "Returns full string definition for message of type '<GetThrusterEfficiency-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetThrusterEfficiency-request)))
  "Returns full string definition for message of type 'GetThrusterEfficiency-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetThrusterEfficiency-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetThrusterEfficiency-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetThrusterEfficiency-request
))
;//! \htmlinclude GetThrusterEfficiency-response.msg.html

(cl:defclass <GetThrusterEfficiency-response> (roslisp-msg-protocol:ros-message)
  ((efficiency
    :reader efficiency
    :initarg :efficiency
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetThrusterEfficiency-response (<GetThrusterEfficiency-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetThrusterEfficiency-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetThrusterEfficiency-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<GetThrusterEfficiency-response> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:GetThrusterEfficiency-response instead.")))

(cl:ensure-generic-function 'efficiency-val :lambda-list '(m))
(cl:defmethod efficiency-val ((m <GetThrusterEfficiency-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:efficiency-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:efficiency instead.")
  (efficiency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetThrusterEfficiency-response>) ostream)
  "Serializes a message object of type '<GetThrusterEfficiency-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'efficiency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetThrusterEfficiency-response>) istream)
  "Deserializes a message object of type '<GetThrusterEfficiency-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'efficiency) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetThrusterEfficiency-response>)))
  "Returns string type for a service object of type '<GetThrusterEfficiency-response>"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiencyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterEfficiency-response)))
  "Returns string type for a service object of type 'GetThrusterEfficiency-response"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiencyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetThrusterEfficiency-response>)))
  "Returns md5sum for a message object of type '<GetThrusterEfficiency-response>"
  "b80ec71e671b93e4cc403df1ac4c8a86")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetThrusterEfficiency-response)))
  "Returns md5sum for a message object of type 'GetThrusterEfficiency-response"
  "b80ec71e671b93e4cc403df1ac4c8a86")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetThrusterEfficiency-response>)))
  "Returns full string definition for message of type '<GetThrusterEfficiency-response>"
  (cl:format cl:nil "float64 efficiency~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetThrusterEfficiency-response)))
  "Returns full string definition for message of type 'GetThrusterEfficiency-response"
  (cl:format cl:nil "float64 efficiency~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetThrusterEfficiency-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetThrusterEfficiency-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetThrusterEfficiency-response
    (cl:cons ':efficiency (efficiency msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetThrusterEfficiency)))
  'GetThrusterEfficiency-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetThrusterEfficiency)))
  'GetThrusterEfficiency-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterEfficiency)))
  "Returns string type for a service object of type '<GetThrusterEfficiency>"
  "uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiency")