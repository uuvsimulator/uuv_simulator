; Auto-generated. Do not edit!


(cl:in-package uuv_thruster_manager-srv)


;//! \htmlinclude ThrusterManagerInfo-request.msg.html

(cl:defclass <ThrusterManagerInfo-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ThrusterManagerInfo-request (<ThrusterManagerInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ThrusterManagerInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ThrusterManagerInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_thruster_manager-srv:<ThrusterManagerInfo-request> is deprecated: use uuv_thruster_manager-srv:ThrusterManagerInfo-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ThrusterManagerInfo-request>) ostream)
  "Serializes a message object of type '<ThrusterManagerInfo-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ThrusterManagerInfo-request>) istream)
  "Deserializes a message object of type '<ThrusterManagerInfo-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ThrusterManagerInfo-request>)))
  "Returns string type for a service object of type '<ThrusterManagerInfo-request>"
  "uuv_thruster_manager/ThrusterManagerInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThrusterManagerInfo-request)))
  "Returns string type for a service object of type 'ThrusterManagerInfo-request"
  "uuv_thruster_manager/ThrusterManagerInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ThrusterManagerInfo-request>)))
  "Returns md5sum for a message object of type '<ThrusterManagerInfo-request>"
  "66fb8ab2f9c5649d97263c955edb636e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ThrusterManagerInfo-request)))
  "Returns md5sum for a message object of type 'ThrusterManagerInfo-request"
  "66fb8ab2f9c5649d97263c955edb636e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ThrusterManagerInfo-request>)))
  "Returns full string definition for message of type '<ThrusterManagerInfo-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ThrusterManagerInfo-request)))
  "Returns full string definition for message of type 'ThrusterManagerInfo-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ThrusterManagerInfo-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ThrusterManagerInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ThrusterManagerInfo-request
))
;//! \htmlinclude ThrusterManagerInfo-response.msg.html

(cl:defclass <ThrusterManagerInfo-response> (roslisp-msg-protocol:ros-message)
  ((n_thrusters
    :reader n_thrusters
    :initarg :n_thrusters
    :type cl:integer
    :initform 0)
   (allocation_matrix
    :reader allocation_matrix
    :initarg :allocation_matrix
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (reference_frame
    :reader reference_frame
    :initarg :reference_frame
    :type cl:string
    :initform ""))
)

(cl:defclass ThrusterManagerInfo-response (<ThrusterManagerInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ThrusterManagerInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ThrusterManagerInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_thruster_manager-srv:<ThrusterManagerInfo-response> is deprecated: use uuv_thruster_manager-srv:ThrusterManagerInfo-response instead.")))

(cl:ensure-generic-function 'n_thrusters-val :lambda-list '(m))
(cl:defmethod n_thrusters-val ((m <ThrusterManagerInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:n_thrusters-val is deprecated.  Use uuv_thruster_manager-srv:n_thrusters instead.")
  (n_thrusters m))

(cl:ensure-generic-function 'allocation_matrix-val :lambda-list '(m))
(cl:defmethod allocation_matrix-val ((m <ThrusterManagerInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:allocation_matrix-val is deprecated.  Use uuv_thruster_manager-srv:allocation_matrix instead.")
  (allocation_matrix m))

(cl:ensure-generic-function 'reference_frame-val :lambda-list '(m))
(cl:defmethod reference_frame-val ((m <ThrusterManagerInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:reference_frame-val is deprecated.  Use uuv_thruster_manager-srv:reference_frame instead.")
  (reference_frame m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ThrusterManagerInfo-response>) ostream)
  "Serializes a message object of type '<ThrusterManagerInfo-response>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reference_frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reference_frame))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ThrusterManagerInfo-response>) istream)
  "Deserializes a message object of type '<ThrusterManagerInfo-response>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reference_frame) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reference_frame) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ThrusterManagerInfo-response>)))
  "Returns string type for a service object of type '<ThrusterManagerInfo-response>"
  "uuv_thruster_manager/ThrusterManagerInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThrusterManagerInfo-response)))
  "Returns string type for a service object of type 'ThrusterManagerInfo-response"
  "uuv_thruster_manager/ThrusterManagerInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ThrusterManagerInfo-response>)))
  "Returns md5sum for a message object of type '<ThrusterManagerInfo-response>"
  "66fb8ab2f9c5649d97263c955edb636e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ThrusterManagerInfo-response)))
  "Returns md5sum for a message object of type 'ThrusterManagerInfo-response"
  "66fb8ab2f9c5649d97263c955edb636e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ThrusterManagerInfo-response>)))
  "Returns full string definition for message of type '<ThrusterManagerInfo-response>"
  (cl:format cl:nil "int32 n_thrusters~%float64[] allocation_matrix~%string reference_frame~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ThrusterManagerInfo-response)))
  "Returns full string definition for message of type 'ThrusterManagerInfo-response"
  (cl:format cl:nil "int32 n_thrusters~%float64[] allocation_matrix~%string reference_frame~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ThrusterManagerInfo-response>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'allocation_matrix) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:length (cl:slot-value msg 'reference_frame))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ThrusterManagerInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ThrusterManagerInfo-response
    (cl:cons ':n_thrusters (n_thrusters msg))
    (cl:cons ':allocation_matrix (allocation_matrix msg))
    (cl:cons ':reference_frame (reference_frame msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ThrusterManagerInfo)))
  'ThrusterManagerInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ThrusterManagerInfo)))
  'ThrusterManagerInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThrusterManagerInfo)))
  "Returns string type for a service object of type '<ThrusterManagerInfo>"
  "uuv_thruster_manager/ThrusterManagerInfo")