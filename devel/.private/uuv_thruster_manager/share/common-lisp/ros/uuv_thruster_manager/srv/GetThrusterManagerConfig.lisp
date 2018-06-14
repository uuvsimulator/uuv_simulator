; Auto-generated. Do not edit!


(cl:in-package uuv_thruster_manager-srv)


;//! \htmlinclude GetThrusterManagerConfig-request.msg.html

(cl:defclass <GetThrusterManagerConfig-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetThrusterManagerConfig-request (<GetThrusterManagerConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetThrusterManagerConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetThrusterManagerConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_thruster_manager-srv:<GetThrusterManagerConfig-request> is deprecated: use uuv_thruster_manager-srv:GetThrusterManagerConfig-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetThrusterManagerConfig-request>) ostream)
  "Serializes a message object of type '<GetThrusterManagerConfig-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetThrusterManagerConfig-request>) istream)
  "Deserializes a message object of type '<GetThrusterManagerConfig-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetThrusterManagerConfig-request>)))
  "Returns string type for a service object of type '<GetThrusterManagerConfig-request>"
  "uuv_thruster_manager/GetThrusterManagerConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterManagerConfig-request)))
  "Returns string type for a service object of type 'GetThrusterManagerConfig-request"
  "uuv_thruster_manager/GetThrusterManagerConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetThrusterManagerConfig-request>)))
  "Returns md5sum for a message object of type '<GetThrusterManagerConfig-request>"
  "b5a2d9d3bb510dd91fdb03f95e95b8de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetThrusterManagerConfig-request)))
  "Returns md5sum for a message object of type 'GetThrusterManagerConfig-request"
  "b5a2d9d3bb510dd91fdb03f95e95b8de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetThrusterManagerConfig-request>)))
  "Returns full string definition for message of type '<GetThrusterManagerConfig-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetThrusterManagerConfig-request)))
  "Returns full string definition for message of type 'GetThrusterManagerConfig-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetThrusterManagerConfig-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetThrusterManagerConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetThrusterManagerConfig-request
))
;//! \htmlinclude GetThrusterManagerConfig-response.msg.html

(cl:defclass <GetThrusterManagerConfig-response> (roslisp-msg-protocol:ros-message)
  ((tf_prefix
    :reader tf_prefix
    :initarg :tf_prefix
    :type cl:string
    :initform "")
   (base_link
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
    :initform 0.0)
   (n_thrusters
    :reader n_thrusters
    :initarg :n_thrusters
    :type cl:integer
    :initform 0)
   (allocation_matrix
    :reader allocation_matrix
    :initarg :allocation_matrix
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetThrusterManagerConfig-response (<GetThrusterManagerConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetThrusterManagerConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetThrusterManagerConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_thruster_manager-srv:<GetThrusterManagerConfig-response> is deprecated: use uuv_thruster_manager-srv:GetThrusterManagerConfig-response instead.")))

(cl:ensure-generic-function 'tf_prefix-val :lambda-list '(m))
(cl:defmethod tf_prefix-val ((m <GetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:tf_prefix-val is deprecated.  Use uuv_thruster_manager-srv:tf_prefix instead.")
  (tf_prefix m))

(cl:ensure-generic-function 'base_link-val :lambda-list '(m))
(cl:defmethod base_link-val ((m <GetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:base_link-val is deprecated.  Use uuv_thruster_manager-srv:base_link instead.")
  (base_link m))

(cl:ensure-generic-function 'thruster_frame_base-val :lambda-list '(m))
(cl:defmethod thruster_frame_base-val ((m <GetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:thruster_frame_base-val is deprecated.  Use uuv_thruster_manager-srv:thruster_frame_base instead.")
  (thruster_frame_base m))

(cl:ensure-generic-function 'thruster_topic_prefix-val :lambda-list '(m))
(cl:defmethod thruster_topic_prefix-val ((m <GetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:thruster_topic_prefix-val is deprecated.  Use uuv_thruster_manager-srv:thruster_topic_prefix instead.")
  (thruster_topic_prefix m))

(cl:ensure-generic-function 'thruster_topic_suffix-val :lambda-list '(m))
(cl:defmethod thruster_topic_suffix-val ((m <GetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:thruster_topic_suffix-val is deprecated.  Use uuv_thruster_manager-srv:thruster_topic_suffix instead.")
  (thruster_topic_suffix m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <GetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:timeout-val is deprecated.  Use uuv_thruster_manager-srv:timeout instead.")
  (timeout m))

(cl:ensure-generic-function 'max_thrust-val :lambda-list '(m))
(cl:defmethod max_thrust-val ((m <GetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:max_thrust-val is deprecated.  Use uuv_thruster_manager-srv:max_thrust instead.")
  (max_thrust m))

(cl:ensure-generic-function 'n_thrusters-val :lambda-list '(m))
(cl:defmethod n_thrusters-val ((m <GetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:n_thrusters-val is deprecated.  Use uuv_thruster_manager-srv:n_thrusters instead.")
  (n_thrusters m))

(cl:ensure-generic-function 'allocation_matrix-val :lambda-list '(m))
(cl:defmethod allocation_matrix-val ((m <GetThrusterManagerConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:allocation_matrix-val is deprecated.  Use uuv_thruster_manager-srv:allocation_matrix instead.")
  (allocation_matrix m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetThrusterManagerConfig-response>) ostream)
  "Serializes a message object of type '<GetThrusterManagerConfig-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tf_prefix))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tf_prefix))
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
  (cl:let* ((signed (cl:slot-value msg 'n_thrusters)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'allocation_matrix))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'allocation_matrix))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetThrusterManagerConfig-response>) istream)
  "Deserializes a message object of type '<GetThrusterManagerConfig-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tf_prefix) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tf_prefix) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_thrusters) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'allocation_matrix) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'allocation_matrix)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetThrusterManagerConfig-response>)))
  "Returns string type for a service object of type '<GetThrusterManagerConfig-response>"
  "uuv_thruster_manager/GetThrusterManagerConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterManagerConfig-response)))
  "Returns string type for a service object of type 'GetThrusterManagerConfig-response"
  "uuv_thruster_manager/GetThrusterManagerConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetThrusterManagerConfig-response>)))
  "Returns md5sum for a message object of type '<GetThrusterManagerConfig-response>"
  "b5a2d9d3bb510dd91fdb03f95e95b8de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetThrusterManagerConfig-response)))
  "Returns md5sum for a message object of type 'GetThrusterManagerConfig-response"
  "b5a2d9d3bb510dd91fdb03f95e95b8de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetThrusterManagerConfig-response>)))
  "Returns full string definition for message of type '<GetThrusterManagerConfig-response>"
  (cl:format cl:nil "string tf_prefix~%string base_link~%string thruster_frame_base~%string thruster_topic_prefix~%string thruster_topic_suffix~%float64 timeout~%float64 max_thrust~%int32 n_thrusters~%float64[] allocation_matrix~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetThrusterManagerConfig-response)))
  "Returns full string definition for message of type 'GetThrusterManagerConfig-response"
  (cl:format cl:nil "string tf_prefix~%string base_link~%string thruster_frame_base~%string thruster_topic_prefix~%string thruster_topic_suffix~%float64 timeout~%float64 max_thrust~%int32 n_thrusters~%float64[] allocation_matrix~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetThrusterManagerConfig-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tf_prefix))
     4 (cl:length (cl:slot-value msg 'base_link))
     4 (cl:length (cl:slot-value msg 'thruster_frame_base))
     4 (cl:length (cl:slot-value msg 'thruster_topic_prefix))
     4 (cl:length (cl:slot-value msg 'thruster_topic_suffix))
     8
     8
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'allocation_matrix) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetThrusterManagerConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetThrusterManagerConfig-response
    (cl:cons ':tf_prefix (tf_prefix msg))
    (cl:cons ':base_link (base_link msg))
    (cl:cons ':thruster_frame_base (thruster_frame_base msg))
    (cl:cons ':thruster_topic_prefix (thruster_topic_prefix msg))
    (cl:cons ':thruster_topic_suffix (thruster_topic_suffix msg))
    (cl:cons ':timeout (timeout msg))
    (cl:cons ':max_thrust (max_thrust msg))
    (cl:cons ':n_thrusters (n_thrusters msg))
    (cl:cons ':allocation_matrix (allocation_matrix msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetThrusterManagerConfig)))
  'GetThrusterManagerConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetThrusterManagerConfig)))
  'GetThrusterManagerConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterManagerConfig)))
  "Returns string type for a service object of type '<GetThrusterManagerConfig>"
  "uuv_thruster_manager/GetThrusterManagerConfig")