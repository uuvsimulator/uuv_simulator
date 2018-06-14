; Auto-generated. Do not edit!


(cl:in-package uuv_thruster_manager-srv)


;//! \htmlinclude SetThrusterManagerConfig-request.msg.html

(cl:defclass <SetThrusterManagerConfig-request> (roslisp-msg-protocol:ros-message)
  ((base_link
    :reader base_link
    :initarg :base_link
    :type cl:string
    :initform "")
   (thruster_frame_base
    :reader thruster_frame_base
    :initarg :thruster_frame_base
    :type cl:string
    :initform "")
   (thruster_topic_prefix
    :reader thruster_topic_prefix
    :initarg :thruster_topic_prefix
    :type cl:string
    :initform "")
   (thruster_topic_suffix
    :reader thruster_topic_suffix
    :initarg :thruster_topic_suffix
    :type cl:string
    :initform "")
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:float
    :initform 0.0)
   (max_thrust
    :reader max_thrust
    :initarg :max_thrust
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetThrusterManagerConfig-request (<SetThrusterManagerConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetThrusterManagerConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetThrusterManagerConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_thruster_manager-srv:<SetThrusterManagerConfig-request> is deprecated: use uuv_thruster_manager-srv:SetThrusterManagerConfig-request instead.")))

(cl:ensure-generic-function 'base_link-val :lambda-list '(m))
(cl:defmethod base_link-val ((m <SetThrusterManagerConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:base_link-val is deprecated.  Use uuv_thruster_manager-srv:base_link instead.")
  (base_link m))

(cl:ensure-generic-function 'thruster_frame_base-val :lambda-list '(m))
(cl:defmethod thruster_frame_base-val ((m <SetThrusterManagerConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:thruster_frame_base-val is deprecated.  Use uuv_thruster_manager-srv:thruster_frame_base instead.")
  (thruster_frame_base m))

(cl:ensure-generic-function 'thruster_topic_prefix-val :lambda-list '(m))
(cl:defmethod thruster_topic_prefix-val ((m <SetThrusterManagerConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:thruster_topic_prefix-val is deprecated.  Use uuv_thruster_manager-srv:thruster_topic_prefix instead.")
  (thruster_topic_prefix m))

(cl:ensure-generic-function 'thruster_topic_suffix-val :lambda-list '(m))
(cl:defmethod thruster_topic_suffix-val ((m <SetThrusterManagerConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:thruster_topic_suffix-val is deprecated.  Use uuv_thruster_manager-srv:thruster_topic_suffix instead.")
  (thruster_topic_suffix m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <SetThrusterManagerConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:timeout-val is deprecated.  Use uuv_thruster_manager-srv:timeout instead.")
  (timeout m))

(cl:ensure-generic-function 'max_thrust-val :lambda-list '(m))
(cl:defmethod max_thrust-val ((m <SetThrusterManagerConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:max_thrust-val is deprecated.  Use uuv_thruster_manager-srv:max_thrust instead.")
  (max_thrust m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SetThrusterManagerConfig-request>)))
    "Constants for message type '<SetThrusterManagerConfig-request>"
  '((:DEFAULT_BASE_LINK . /base_link)
    (:DEFAULT_THRUSTER_FRAME_BASE . /thruster_)
    (:DEFAULT_PREFIX . thrusters/)
    (:DEFAULT_SUFFIX . /input))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SetThrusterManagerConfig-request)))
    "Constants for message type 'SetThrusterManagerConfig-request"
  '((:DEFAULT_BASE_LINK . /base_link)
    (:DEFAULT_THRUSTER_FRAME_BASE . /thruster_)
    (:DEFAULT_PREFIX . thrusters/)
    (:DEFAULT_SUFFIX . /input))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetThrusterManagerConfig-request>) ostream)
  "Serializes a message object of type '<SetThrusterManagerConfig-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'base_link))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'base_link))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'thruster_frame_base))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'thruster_frame_base))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'thruster_topic_prefix))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'thruster_topic_prefix))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'thruster_topic_suffix))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'thruster_topic_suffix))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timeout))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetThrusterManagerConfig-request>) istream)
  "Deserializes a message object of type '<SetThrusterManagerConfig-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'base_link) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'base_link) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'thruster_frame_base) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'thruster_frame_base) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'thruster_topic_prefix) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'thruster_topic_prefix) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'thruster_topic_suffix) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'thruster_topic_suffix) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeout) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_thrust) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetThrusterManagerConfig-request>)))
  "Returns string type for a service object of type '<SetThrusterManagerConfig-request>"
  "uuv_thruster_manager/SetThrusterManagerConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThrusterManagerConfig-request)))
  "Returns string type for a service object of type 'SetThrusterManagerConfig-request"
  "uuv_thruster_manager/SetThrusterManagerConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetThrusterManagerConfig-request>)))
  "Returns md5sum for a message object of type '<SetThrusterManagerConfig-request>"
  "e9f260f9f8a74cbd9cca6d6d276790c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetThrusterManagerConfig-request)))
  "Returns md5sum for a message object of type 'SetThrusterManagerConfig-request"
  "e9f260f9f8a74cbd9cca6d6d276790c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetThrusterManagerConfig-request>)))
  "Returns full string definition for message of type '<SetThrusterManagerConfig-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string DEFAULT_BASE_LINK            = /base_link~%string DEFAULT_THRUSTER_FRAME_BASE  = /thruster_~%string DEFAULT_PREFIX               = thrusters/~%string DEFAULT_SUFFIX               = /input~%~%string base_link~%string thruster_frame_base~%string thruster_topic_prefix~%string thruster_topic_suffix~%float64 timeout~%float64 max_thrust~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetThrusterManagerConfig-request)))
  "Returns full string definition for message of type 'SetThrusterManagerConfig-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string DEFAULT_BASE_LINK            = /base_link~%string DEFAULT_THRUSTER_FRAME_BASE  = /thruster_~%string DEFAULT_PREFIX               = thrusters/~%string DEFAULT_SUFFIX               = /input~%~%string base_link~%string thruster_frame_base~%string thruster_topic_prefix~%string thruster_topic_suffix~%float64 timeout~%float64 max_thrust~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetThrusterManagerConfig-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'base_link))
     4 (cl:length (cl:slot-value msg 'thruster_frame_base))
     4 (cl:length (cl:slot-value msg 'thruster_topic_prefix))
     4 (cl:length (cl:slot-value msg 'thruster_topic_suffix))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetThrusterManagerConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetThrusterManagerConfig-request
    (cl:cons ':base_link (base_link msg))
    (cl:cons ':thruster_frame_base (thruster_frame_base msg))
    (cl:cons ':thruster_topic_prefix (thruster_topic_prefix msg))
    (cl:cons ':thruster_topic_suffix (thruster_topic_suffix msg))
    (cl:cons ':timeout (timeout msg))
    (cl:cons ':max_thrust (max_thrust msg))
))
;//! \htmlinclude SetThrusterManagerConfig-response.msg.html

(cl:defclass <SetThrusterManagerConfig-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetThrusterManagerConfig-response (<SetThrusterManagerConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetThrusterManagerConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetThrusterManagerConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_thruster_manager-srv:<SetThrusterManagerConfig-response> is deprecated: use uuv_thruster_manager-srv:SetThrusterManagerConfig-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:success-val is deprecated.  Use uuv_thruster_manager-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetThrusterManagerConfig-response>) ostream)
  "Serializes a message object of type '<SetThrusterManagerConfig-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetThrusterManagerConfig-response>) istream)
  "Deserializes a message object of type '<SetThrusterManagerConfig-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetThrusterManagerConfig-response>)))
  "Returns string type for a service object of type '<SetThrusterManagerConfig-response>"
  "uuv_thruster_manager/SetThrusterManagerConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThrusterManagerConfig-response)))
  "Returns string type for a service object of type 'SetThrusterManagerConfig-response"
  "uuv_thruster_manager/SetThrusterManagerConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetThrusterManagerConfig-response>)))
  "Returns md5sum for a message object of type '<SetThrusterManagerConfig-response>"
  "e9f260f9f8a74cbd9cca6d6d276790c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetThrusterManagerConfig-response)))
  "Returns md5sum for a message object of type 'SetThrusterManagerConfig-response"
  "e9f260f9f8a74cbd9cca6d6d276790c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetThrusterManagerConfig-response>)))
  "Returns full string definition for message of type '<SetThrusterManagerConfig-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetThrusterManagerConfig-response)))
  "Returns full string definition for message of type 'SetThrusterManagerConfig-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetThrusterManagerConfig-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetThrusterManagerConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetThrusterManagerConfig-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetThrusterManagerConfig)))
  'SetThrusterManagerConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetThrusterManagerConfig)))
  'SetThrusterManagerConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThrusterManagerConfig)))
  "Returns string type for a service object of type '<SetThrusterManagerConfig>"
  "uuv_thruster_manager/SetThrusterManagerConfig")