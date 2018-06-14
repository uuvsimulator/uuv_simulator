; Auto-generated. Do not edit!


(cl:in-package uuv_world_ros_plugins_msgs-srv)


;//! \htmlinclude GetCurrentModel-request.msg.html

(cl:defclass <GetCurrentModel-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetCurrentModel-request (<GetCurrentModel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentModel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentModel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<GetCurrentModel-request> is deprecated: use uuv_world_ros_plugins_msgs-srv:GetCurrentModel-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentModel-request>) ostream)
  "Serializes a message object of type '<GetCurrentModel-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentModel-request>) istream)
  "Deserializes a message object of type '<GetCurrentModel-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentModel-request>)))
  "Returns string type for a service object of type '<GetCurrentModel-request>"
  "uuv_world_ros_plugins_msgs/GetCurrentModelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentModel-request)))
  "Returns string type for a service object of type 'GetCurrentModel-request"
  "uuv_world_ros_plugins_msgs/GetCurrentModelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentModel-request>)))
  "Returns md5sum for a message object of type '<GetCurrentModel-request>"
  "b8222571af4e4180b9b706d1e17ad7e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentModel-request)))
  "Returns md5sum for a message object of type 'GetCurrentModel-request"
  "b8222571af4e4180b9b706d1e17ad7e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentModel-request>)))
  "Returns full string definition for message of type '<GetCurrentModel-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentModel-request)))
  "Returns full string definition for message of type 'GetCurrentModel-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentModel-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentModel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentModel-request
))
;//! \htmlinclude GetCurrentModel-response.msg.html

(cl:defclass <GetCurrentModel-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetCurrentModel-response (<GetCurrentModel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentModel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentModel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<GetCurrentModel-response> is deprecated: use uuv_world_ros_plugins_msgs-srv:GetCurrentModel-response instead.")))

(cl:ensure-generic-function 'mean-val :lambda-list '(m))
(cl:defmethod mean-val ((m <GetCurrentModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:mean-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:mean instead.")
  (mean m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <GetCurrentModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:min-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <GetCurrentModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:max-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:max instead.")
  (max m))

(cl:ensure-generic-function 'noise-val :lambda-list '(m))
(cl:defmethod noise-val ((m <GetCurrentModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:noise-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:noise instead.")
  (noise m))

(cl:ensure-generic-function 'mu-val :lambda-list '(m))
(cl:defmethod mu-val ((m <GetCurrentModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:mu-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:mu instead.")
  (mu m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentModel-response>) ostream)
  "Serializes a message object of type '<GetCurrentModel-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentModel-response>) istream)
  "Deserializes a message object of type '<GetCurrentModel-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentModel-response>)))
  "Returns string type for a service object of type '<GetCurrentModel-response>"
  "uuv_world_ros_plugins_msgs/GetCurrentModelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentModel-response)))
  "Returns string type for a service object of type 'GetCurrentModel-response"
  "uuv_world_ros_plugins_msgs/GetCurrentModelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentModel-response>)))
  "Returns md5sum for a message object of type '<GetCurrentModel-response>"
  "b8222571af4e4180b9b706d1e17ad7e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentModel-response)))
  "Returns md5sum for a message object of type 'GetCurrentModel-response"
  "b8222571af4e4180b9b706d1e17ad7e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentModel-response>)))
  "Returns full string definition for message of type '<GetCurrentModel-response>"
  (cl:format cl:nil "float64 mean~%float64 min~%float64 max~%float64 noise~%float64 mu~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentModel-response)))
  "Returns full string definition for message of type 'GetCurrentModel-response"
  (cl:format cl:nil "float64 mean~%float64 min~%float64 max~%float64 noise~%float64 mu~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentModel-response>))
  (cl:+ 0
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentModel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentModel-response
    (cl:cons ':mean (mean msg))
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
    (cl:cons ':noise (noise msg))
    (cl:cons ':mu (mu msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentModel)))
  'GetCurrentModel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentModel)))
  'GetCurrentModel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentModel)))
  "Returns string type for a service object of type '<GetCurrentModel>"
  "uuv_world_ros_plugins_msgs/GetCurrentModel")