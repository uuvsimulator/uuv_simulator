; Auto-generated. Do not edit!


(cl:in-package uuv_world_ros_plugins_msgs-srv)


;//! \htmlinclude SetCurrentDirection-request.msg.html

(cl:defclass <SetCurrentDirection-request> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetCurrentDirection-request (<SetCurrentDirection-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCurrentDirection-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCurrentDirection-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<SetCurrentDirection-request> is deprecated: use uuv_world_ros_plugins_msgs-srv:SetCurrentDirection-request instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <SetCurrentDirection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:angle-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCurrentDirection-request>) ostream)
  "Serializes a message object of type '<SetCurrentDirection-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCurrentDirection-request>) istream)
  "Deserializes a message object of type '<SetCurrentDirection-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCurrentDirection-request>)))
  "Returns string type for a service object of type '<SetCurrentDirection-request>"
  "uuv_world_ros_plugins_msgs/SetCurrentDirectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentDirection-request)))
  "Returns string type for a service object of type 'SetCurrentDirection-request"
  "uuv_world_ros_plugins_msgs/SetCurrentDirectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCurrentDirection-request>)))
  "Returns md5sum for a message object of type '<SetCurrentDirection-request>"
  "c1a76fcaf62dc4534903e93216b59a79")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCurrentDirection-request)))
  "Returns md5sum for a message object of type 'SetCurrentDirection-request"
  "c1a76fcaf62dc4534903e93216b59a79")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCurrentDirection-request>)))
  "Returns full string definition for message of type '<SetCurrentDirection-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCurrentDirection-request)))
  "Returns full string definition for message of type 'SetCurrentDirection-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCurrentDirection-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCurrentDirection-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCurrentDirection-request
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude SetCurrentDirection-response.msg.html

(cl:defclass <SetCurrentDirection-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetCurrentDirection-response (<SetCurrentDirection-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCurrentDirection-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCurrentDirection-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<SetCurrentDirection-response> is deprecated: use uuv_world_ros_plugins_msgs-srv:SetCurrentDirection-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetCurrentDirection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:success-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCurrentDirection-response>) ostream)
  "Serializes a message object of type '<SetCurrentDirection-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCurrentDirection-response>) istream)
  "Deserializes a message object of type '<SetCurrentDirection-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCurrentDirection-response>)))
  "Returns string type for a service object of type '<SetCurrentDirection-response>"
  "uuv_world_ros_plugins_msgs/SetCurrentDirectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentDirection-response)))
  "Returns string type for a service object of type 'SetCurrentDirection-response"
  "uuv_world_ros_plugins_msgs/SetCurrentDirectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCurrentDirection-response>)))
  "Returns md5sum for a message object of type '<SetCurrentDirection-response>"
  "c1a76fcaf62dc4534903e93216b59a79")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCurrentDirection-response)))
  "Returns md5sum for a message object of type 'SetCurrentDirection-response"
  "c1a76fcaf62dc4534903e93216b59a79")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCurrentDirection-response>)))
  "Returns full string definition for message of type '<SetCurrentDirection-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCurrentDirection-response)))
  "Returns full string definition for message of type 'SetCurrentDirection-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCurrentDirection-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCurrentDirection-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCurrentDirection-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCurrentDirection)))
  'SetCurrentDirection-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCurrentDirection)))
  'SetCurrentDirection-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentDirection)))
  "Returns string type for a service object of type '<SetCurrentDirection>"
  "uuv_world_ros_plugins_msgs/SetCurrentDirection")