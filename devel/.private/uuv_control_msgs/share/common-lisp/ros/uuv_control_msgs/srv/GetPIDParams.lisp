; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude GetPIDParams-request.msg.html

(cl:defclass <GetPIDParams-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetPIDParams-request (<GetPIDParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPIDParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPIDParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<GetPIDParams-request> is deprecated: use uuv_control_msgs-srv:GetPIDParams-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPIDParams-request>) ostream)
  "Serializes a message object of type '<GetPIDParams-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPIDParams-request>) istream)
  "Deserializes a message object of type '<GetPIDParams-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPIDParams-request>)))
  "Returns string type for a service object of type '<GetPIDParams-request>"
  "uuv_control_msgs/GetPIDParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPIDParams-request)))
  "Returns string type for a service object of type 'GetPIDParams-request"
  "uuv_control_msgs/GetPIDParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPIDParams-request>)))
  "Returns md5sum for a message object of type '<GetPIDParams-request>"
  "1dae001799e4bc231c788fb194cf733a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPIDParams-request)))
  "Returns md5sum for a message object of type 'GetPIDParams-request"
  "1dae001799e4bc231c788fb194cf733a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPIDParams-request>)))
  "Returns full string definition for message of type '<GetPIDParams-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPIDParams-request)))
  "Returns full string definition for message of type 'GetPIDParams-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPIDParams-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPIDParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPIDParams-request
))
;//! \htmlinclude GetPIDParams-response.msg.html

(cl:defclass <GetPIDParams-response> (roslisp-msg-protocol:ros-message)
  ((Kp
    :reader Kp
    :initarg :Kp
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (Kd
    :reader Kd
    :initarg :Kd
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (Ki
    :reader Ki
    :initarg :Ki
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetPIDParams-response (<GetPIDParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPIDParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPIDParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<GetPIDParams-response> is deprecated: use uuv_control_msgs-srv:GetPIDParams-response instead.")))

(cl:ensure-generic-function 'Kp-val :lambda-list '(m))
(cl:defmethod Kp-val ((m <GetPIDParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:Kp-val is deprecated.  Use uuv_control_msgs-srv:Kp instead.")
  (Kp m))

(cl:ensure-generic-function 'Kd-val :lambda-list '(m))
(cl:defmethod Kd-val ((m <GetPIDParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:Kd-val is deprecated.  Use uuv_control_msgs-srv:Kd instead.")
  (Kd m))

(cl:ensure-generic-function 'Ki-val :lambda-list '(m))
(cl:defmethod Ki-val ((m <GetPIDParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:Ki-val is deprecated.  Use uuv_control_msgs-srv:Ki instead.")
  (Ki m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPIDParams-response>) ostream)
  "Serializes a message object of type '<GetPIDParams-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Kp))))
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
   (cl:slot-value msg 'Kp))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Kd))))
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
   (cl:slot-value msg 'Kd))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Ki))))
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
   (cl:slot-value msg 'Ki))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPIDParams-response>) istream)
  "Deserializes a message object of type '<GetPIDParams-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Kp) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Kp)))
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Kd) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Kd)))
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Ki) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Ki)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPIDParams-response>)))
  "Returns string type for a service object of type '<GetPIDParams-response>"
  "uuv_control_msgs/GetPIDParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPIDParams-response)))
  "Returns string type for a service object of type 'GetPIDParams-response"
  "uuv_control_msgs/GetPIDParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPIDParams-response>)))
  "Returns md5sum for a message object of type '<GetPIDParams-response>"
  "1dae001799e4bc231c788fb194cf733a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPIDParams-response)))
  "Returns md5sum for a message object of type 'GetPIDParams-response"
  "1dae001799e4bc231c788fb194cf733a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPIDParams-response>)))
  "Returns full string definition for message of type '<GetPIDParams-response>"
  (cl:format cl:nil "float64[] Kp~%float64[] Kd~%float64[] Ki~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPIDParams-response)))
  "Returns full string definition for message of type 'GetPIDParams-response"
  (cl:format cl:nil "float64[] Kp~%float64[] Kd~%float64[] Ki~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPIDParams-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Kp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Kd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Ki) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPIDParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPIDParams-response
    (cl:cons ':Kp (Kp msg))
    (cl:cons ':Kd (Kd msg))
    (cl:cons ':Ki (Ki msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPIDParams)))
  'GetPIDParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPIDParams)))
  'GetPIDParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPIDParams)))
  "Returns string type for a service object of type '<GetPIDParams>"
  "uuv_control_msgs/GetPIDParams")