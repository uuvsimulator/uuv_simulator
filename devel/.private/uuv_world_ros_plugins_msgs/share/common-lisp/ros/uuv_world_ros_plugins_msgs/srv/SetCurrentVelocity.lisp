; Auto-generated. Do not edit!


(cl:in-package uuv_world_ros_plugins_msgs-srv)


;//! \htmlinclude SetCurrentVelocity-request.msg.html

(cl:defclass <SetCurrentVelocity-request> (roslisp-msg-protocol:ros-message)
  ((velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (horizontal_angle
    :reader horizontal_angle
    :initarg :horizontal_angle
    :type cl:float
    :initform 0.0)
   (vertical_angle
    :reader vertical_angle
    :initarg :vertical_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetCurrentVelocity-request (<SetCurrentVelocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCurrentVelocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCurrentVelocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<SetCurrentVelocity-request> is deprecated: use uuv_world_ros_plugins_msgs-srv:SetCurrentVelocity-request instead.")))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <SetCurrentVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:velocity-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'horizontal_angle-val :lambda-list '(m))
(cl:defmethod horizontal_angle-val ((m <SetCurrentVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:horizontal_angle-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:horizontal_angle instead.")
  (horizontal_angle m))

(cl:ensure-generic-function 'vertical_angle-val :lambda-list '(m))
(cl:defmethod vertical_angle-val ((m <SetCurrentVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:vertical_angle-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:vertical_angle instead.")
  (vertical_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCurrentVelocity-request>) ostream)
  "Serializes a message object of type '<SetCurrentVelocity-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'horizontal_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vertical_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCurrentVelocity-request>) istream)
  "Deserializes a message object of type '<SetCurrentVelocity-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horizontal_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vertical_angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCurrentVelocity-request>)))
  "Returns string type for a service object of type '<SetCurrentVelocity-request>"
  "uuv_world_ros_plugins_msgs/SetCurrentVelocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentVelocity-request)))
  "Returns string type for a service object of type 'SetCurrentVelocity-request"
  "uuv_world_ros_plugins_msgs/SetCurrentVelocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCurrentVelocity-request>)))
  "Returns md5sum for a message object of type '<SetCurrentVelocity-request>"
  "3389770cff5466e5c98d6200f7909bd7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCurrentVelocity-request)))
  "Returns md5sum for a message object of type 'SetCurrentVelocity-request"
  "3389770cff5466e5c98d6200f7909bd7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCurrentVelocity-request>)))
  "Returns full string definition for message of type '<SetCurrentVelocity-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 velocity~%float64 horizontal_angle~%float64 vertical_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCurrentVelocity-request)))
  "Returns full string definition for message of type 'SetCurrentVelocity-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 velocity~%float64 horizontal_angle~%float64 vertical_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCurrentVelocity-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCurrentVelocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCurrentVelocity-request
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':horizontal_angle (horizontal_angle msg))
    (cl:cons ':vertical_angle (vertical_angle msg))
))
;//! \htmlinclude SetCurrentVelocity-response.msg.html

(cl:defclass <SetCurrentVelocity-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetCurrentVelocity-response (<SetCurrentVelocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCurrentVelocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCurrentVelocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<SetCurrentVelocity-response> is deprecated: use uuv_world_ros_plugins_msgs-srv:SetCurrentVelocity-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetCurrentVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:success-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCurrentVelocity-response>) ostream)
  "Serializes a message object of type '<SetCurrentVelocity-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCurrentVelocity-response>) istream)
  "Deserializes a message object of type '<SetCurrentVelocity-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCurrentVelocity-response>)))
  "Returns string type for a service object of type '<SetCurrentVelocity-response>"
  "uuv_world_ros_plugins_msgs/SetCurrentVelocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentVelocity-response)))
  "Returns string type for a service object of type 'SetCurrentVelocity-response"
  "uuv_world_ros_plugins_msgs/SetCurrentVelocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCurrentVelocity-response>)))
  "Returns md5sum for a message object of type '<SetCurrentVelocity-response>"
  "3389770cff5466e5c98d6200f7909bd7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCurrentVelocity-response)))
  "Returns md5sum for a message object of type 'SetCurrentVelocity-response"
  "3389770cff5466e5c98d6200f7909bd7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCurrentVelocity-response>)))
  "Returns full string definition for message of type '<SetCurrentVelocity-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCurrentVelocity-response)))
  "Returns full string definition for message of type 'SetCurrentVelocity-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCurrentVelocity-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCurrentVelocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCurrentVelocity-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCurrentVelocity)))
  'SetCurrentVelocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCurrentVelocity)))
  'SetCurrentVelocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentVelocity)))
  "Returns string type for a service object of type '<SetCurrentVelocity>"
  "uuv_world_ros_plugins_msgs/SetCurrentVelocity")