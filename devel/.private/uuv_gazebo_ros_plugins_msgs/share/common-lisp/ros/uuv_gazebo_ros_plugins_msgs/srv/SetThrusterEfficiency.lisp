; Auto-generated. Do not edit!


(cl:in-package uuv_gazebo_ros_plugins_msgs-srv)


;//! \htmlinclude SetThrusterEfficiency-request.msg.html

(cl:defclass <SetThrusterEfficiency-request> (roslisp-msg-protocol:ros-message)
  ((efficiency
    :reader efficiency
    :initarg :efficiency
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetThrusterEfficiency-request (<SetThrusterEfficiency-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetThrusterEfficiency-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetThrusterEfficiency-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<SetThrusterEfficiency-request> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:SetThrusterEfficiency-request instead.")))

(cl:ensure-generic-function 'efficiency-val :lambda-list '(m))
(cl:defmethod efficiency-val ((m <SetThrusterEfficiency-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:efficiency-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:efficiency instead.")
  (efficiency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetThrusterEfficiency-request>) ostream)
  "Serializes a message object of type '<SetThrusterEfficiency-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetThrusterEfficiency-request>) istream)
  "Deserializes a message object of type '<SetThrusterEfficiency-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetThrusterEfficiency-request>)))
  "Returns string type for a service object of type '<SetThrusterEfficiency-request>"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiencyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThrusterEfficiency-request)))
  "Returns string type for a service object of type 'SetThrusterEfficiency-request"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiencyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetThrusterEfficiency-request>)))
  "Returns md5sum for a message object of type '<SetThrusterEfficiency-request>"
  "60f827235457ddfd6a19b596030b49ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetThrusterEfficiency-request)))
  "Returns md5sum for a message object of type 'SetThrusterEfficiency-request"
  "60f827235457ddfd6a19b596030b49ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetThrusterEfficiency-request>)))
  "Returns full string definition for message of type '<SetThrusterEfficiency-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 efficiency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetThrusterEfficiency-request)))
  "Returns full string definition for message of type 'SetThrusterEfficiency-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 efficiency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetThrusterEfficiency-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetThrusterEfficiency-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetThrusterEfficiency-request
    (cl:cons ':efficiency (efficiency msg))
))
;//! \htmlinclude SetThrusterEfficiency-response.msg.html

(cl:defclass <SetThrusterEfficiency-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetThrusterEfficiency-response (<SetThrusterEfficiency-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetThrusterEfficiency-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetThrusterEfficiency-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_gazebo_ros_plugins_msgs-srv:<SetThrusterEfficiency-response> is deprecated: use uuv_gazebo_ros_plugins_msgs-srv:SetThrusterEfficiency-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetThrusterEfficiency-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_gazebo_ros_plugins_msgs-srv:success-val is deprecated.  Use uuv_gazebo_ros_plugins_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetThrusterEfficiency-response>) ostream)
  "Serializes a message object of type '<SetThrusterEfficiency-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetThrusterEfficiency-response>) istream)
  "Deserializes a message object of type '<SetThrusterEfficiency-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetThrusterEfficiency-response>)))
  "Returns string type for a service object of type '<SetThrusterEfficiency-response>"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiencyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThrusterEfficiency-response)))
  "Returns string type for a service object of type 'SetThrusterEfficiency-response"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiencyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetThrusterEfficiency-response>)))
  "Returns md5sum for a message object of type '<SetThrusterEfficiency-response>"
  "60f827235457ddfd6a19b596030b49ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetThrusterEfficiency-response)))
  "Returns md5sum for a message object of type 'SetThrusterEfficiency-response"
  "60f827235457ddfd6a19b596030b49ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetThrusterEfficiency-response>)))
  "Returns full string definition for message of type '<SetThrusterEfficiency-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetThrusterEfficiency-response)))
  "Returns full string definition for message of type 'SetThrusterEfficiency-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetThrusterEfficiency-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetThrusterEfficiency-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetThrusterEfficiency-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetThrusterEfficiency)))
  'SetThrusterEfficiency-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetThrusterEfficiency)))
  'SetThrusterEfficiency-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThrusterEfficiency)))
  "Returns string type for a service object of type '<SetThrusterEfficiency>"
  "uuv_gazebo_ros_plugins_msgs/SetThrusterEfficiency")