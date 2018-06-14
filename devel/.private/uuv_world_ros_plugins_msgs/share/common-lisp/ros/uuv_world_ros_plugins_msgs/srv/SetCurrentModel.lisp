; Auto-generated. Do not edit!


(cl:in-package uuv_world_ros_plugins_msgs-srv)


;//! \htmlinclude SetCurrentModel-request.msg.html

(cl:defclass <SetCurrentModel-request> (roslisp-msg-protocol:ros-message)
  ((mean
    :reader mean
    :initarg :mean
    :type cl:float
    :initform 0.0)
   (min
    :reader min
    :initarg :min
    :type cl:float
    :initform 0.0)
   (max
    :reader max
    :initarg :max
    :type cl:float
    :initform 0.0)
   (noise
    :reader noise
    :initarg :noise
    :type cl:float
    :initform 0.0)
   (mu
    :reader mu
    :initarg :mu
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetCurrentModel-request (<SetCurrentModel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCurrentModel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCurrentModel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<SetCurrentModel-request> is deprecated: use uuv_world_ros_plugins_msgs-srv:SetCurrentModel-request instead.")))

(cl:ensure-generic-function 'mean-val :lambda-list '(m))
(cl:defmethod mean-val ((m <SetCurrentModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:mean-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:mean instead.")
  (mean m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <SetCurrentModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:min-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <SetCurrentModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:max-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:max instead.")
  (max m))

(cl:ensure-generic-function 'noise-val :lambda-list '(m))
(cl:defmethod noise-val ((m <SetCurrentModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:noise-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:noise instead.")
  (noise m))

(cl:ensure-generic-function 'mu-val :lambda-list '(m))
(cl:defmethod mu-val ((m <SetCurrentModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:mu-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:mu instead.")
  (mu m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCurrentModel-request>) ostream)
  "Serializes a message object of type '<SetCurrentModel-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'noise))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCurrentModel-request>) istream)
  "Deserializes a message object of type '<SetCurrentModel-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mean) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'noise) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mu) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCurrentModel-request>)))
  "Returns string type for a service object of type '<SetCurrentModel-request>"
  "uuv_world_ros_plugins_msgs/SetCurrentModelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentModel-request)))
  "Returns string type for a service object of type 'SetCurrentModel-request"
  "uuv_world_ros_plugins_msgs/SetCurrentModelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCurrentModel-request>)))
  "Returns md5sum for a message object of type '<SetCurrentModel-request>"
  "97b431fd7a0d7100472976b98c315e14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCurrentModel-request)))
  "Returns md5sum for a message object of type 'SetCurrentModel-request"
  "97b431fd7a0d7100472976b98c315e14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCurrentModel-request>)))
  "Returns full string definition for message of type '<SetCurrentModel-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 mean~%float64 min~%float64 max~%float64 noise~%float64 mu~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCurrentModel-request)))
  "Returns full string definition for message of type 'SetCurrentModel-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 mean~%float64 min~%float64 max~%float64 noise~%float64 mu~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCurrentModel-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCurrentModel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCurrentModel-request
    (cl:cons ':mean (mean msg))
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
    (cl:cons ':noise (noise msg))
    (cl:cons ':mu (mu msg))
))
;//! \htmlinclude SetCurrentModel-response.msg.html

(cl:defclass <SetCurrentModel-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetCurrentModel-response (<SetCurrentModel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCurrentModel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCurrentModel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<SetCurrentModel-response> is deprecated: use uuv_world_ros_plugins_msgs-srv:SetCurrentModel-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetCurrentModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:success-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCurrentModel-response>) ostream)
  "Serializes a message object of type '<SetCurrentModel-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCurrentModel-response>) istream)
  "Deserializes a message object of type '<SetCurrentModel-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCurrentModel-response>)))
  "Returns string type for a service object of type '<SetCurrentModel-response>"
  "uuv_world_ros_plugins_msgs/SetCurrentModelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentModel-response)))
  "Returns string type for a service object of type 'SetCurrentModel-response"
  "uuv_world_ros_plugins_msgs/SetCurrentModelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCurrentModel-response>)))
  "Returns md5sum for a message object of type '<SetCurrentModel-response>"
  "97b431fd7a0d7100472976b98c315e14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCurrentModel-response)))
  "Returns md5sum for a message object of type 'SetCurrentModel-response"
  "97b431fd7a0d7100472976b98c315e14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCurrentModel-response>)))
  "Returns full string definition for message of type '<SetCurrentModel-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCurrentModel-response)))
  "Returns full string definition for message of type 'SetCurrentModel-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCurrentModel-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCurrentModel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCurrentModel-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCurrentModel)))
  'SetCurrentModel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCurrentModel)))
  'SetCurrentModel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentModel)))
  "Returns string type for a service object of type '<SetCurrentModel>"
  "uuv_world_ros_plugins_msgs/SetCurrentModel")