; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude GoToIncremental-request.msg.html

(cl:defclass <GoToIncremental-request> (roslisp-msg-protocol:ros-message)
  ((step
    :reader step
    :initarg :step
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (max_forward_speed
    :reader max_forward_speed
    :initarg :max_forward_speed
    :type cl:float
    :initform 0.0)
   (interpolator
    :reader interpolator
    :initarg :interpolator
    :type cl:string
    :initform ""))
)

(cl:defclass GoToIncremental-request (<GoToIncremental-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToIncremental-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToIncremental-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<GoToIncremental-request> is deprecated: use uuv_control_msgs-srv:GoToIncremental-request instead.")))

(cl:ensure-generic-function 'step-val :lambda-list '(m))
(cl:defmethod step-val ((m <GoToIncremental-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:step-val is deprecated.  Use uuv_control_msgs-srv:step instead.")
  (step m))

(cl:ensure-generic-function 'max_forward_speed-val :lambda-list '(m))
(cl:defmethod max_forward_speed-val ((m <GoToIncremental-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:max_forward_speed-val is deprecated.  Use uuv_control_msgs-srv:max_forward_speed instead.")
  (max_forward_speed m))

(cl:ensure-generic-function 'interpolator-val :lambda-list '(m))
(cl:defmethod interpolator-val ((m <GoToIncremental-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:interpolator-val is deprecated.  Use uuv_control_msgs-srv:interpolator instead.")
  (interpolator m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GoToIncremental-request>)))
    "Constants for message type '<GoToIncremental-request>"
  '((:LIPB . 'lipb')
    (:CUBIC . 'cubic')
    (:DUBINS . 'dubins')
    (:LINEAR . 'linear'))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GoToIncremental-request)))
    "Constants for message type 'GoToIncremental-request"
  '((:LIPB . 'lipb')
    (:CUBIC . 'cubic')
    (:DUBINS . 'dubins')
    (:LINEAR . 'linear'))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToIncremental-request>) ostream)
  "Serializes a message object of type '<GoToIncremental-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'step) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_forward_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'interpolator))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'interpolator))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToIncremental-request>) istream)
  "Deserializes a message object of type '<GoToIncremental-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'step) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_forward_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'interpolator) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'interpolator) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToIncremental-request>)))
  "Returns string type for a service object of type '<GoToIncremental-request>"
  "uuv_control_msgs/GoToIncrementalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToIncremental-request)))
  "Returns string type for a service object of type 'GoToIncremental-request"
  "uuv_control_msgs/GoToIncrementalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToIncremental-request>)))
  "Returns md5sum for a message object of type '<GoToIncremental-request>"
  "ea062c779dd21cbac8fefabcd9b5f18e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToIncremental-request)))
  "Returns md5sum for a message object of type 'GoToIncremental-request"
  "ea062c779dd21cbac8fefabcd9b5f18e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToIncremental-request>)))
  "Returns full string definition for message of type '<GoToIncremental-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string LIPB='lipb'~%string CUBIC='cubic'~%string DUBINS='dubins'~%string LINEAR='linear'~%~%geometry_msgs/Point step~%float64 max_forward_speed~%string interpolator~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToIncremental-request)))
  "Returns full string definition for message of type 'GoToIncremental-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string LIPB='lipb'~%string CUBIC='cubic'~%string DUBINS='dubins'~%string LINEAR='linear'~%~%geometry_msgs/Point step~%float64 max_forward_speed~%string interpolator~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToIncremental-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'step))
     8
     4 (cl:length (cl:slot-value msg 'interpolator))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToIncremental-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToIncremental-request
    (cl:cons ':step (step msg))
    (cl:cons ':max_forward_speed (max_forward_speed msg))
    (cl:cons ':interpolator (interpolator msg))
))
;//! \htmlinclude GoToIncremental-response.msg.html

(cl:defclass <GoToIncremental-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GoToIncremental-response (<GoToIncremental-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToIncremental-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToIncremental-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<GoToIncremental-response> is deprecated: use uuv_control_msgs-srv:GoToIncremental-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GoToIncremental-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToIncremental-response>) ostream)
  "Serializes a message object of type '<GoToIncremental-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToIncremental-response>) istream)
  "Deserializes a message object of type '<GoToIncremental-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToIncremental-response>)))
  "Returns string type for a service object of type '<GoToIncremental-response>"
  "uuv_control_msgs/GoToIncrementalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToIncremental-response)))
  "Returns string type for a service object of type 'GoToIncremental-response"
  "uuv_control_msgs/GoToIncrementalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToIncremental-response>)))
  "Returns md5sum for a message object of type '<GoToIncremental-response>"
  "ea062c779dd21cbac8fefabcd9b5f18e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToIncremental-response)))
  "Returns md5sum for a message object of type 'GoToIncremental-response"
  "ea062c779dd21cbac8fefabcd9b5f18e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToIncremental-response>)))
  "Returns full string definition for message of type '<GoToIncremental-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToIncremental-response)))
  "Returns full string definition for message of type 'GoToIncremental-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToIncremental-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToIncremental-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToIncremental-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoToIncremental)))
  'GoToIncremental-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoToIncremental)))
  'GoToIncremental-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToIncremental)))
  "Returns string type for a service object of type '<GoToIncremental>"
  "uuv_control_msgs/GoToIncremental")