; Auto-generated. Do not edit!


(cl:in-package uuv_manipulators_msgs-msg)


;//! \htmlinclude EndeffectorCommand.msg.html

(cl:defclass <EndeffectorCommand> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform "")
   (ratio
    :reader ratio
    :initarg :ratio
    :type cl:float
    :initform 0.0))
)

(cl:defclass EndeffectorCommand (<EndeffectorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EndeffectorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EndeffectorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_manipulators_msgs-msg:<EndeffectorCommand> is deprecated: use uuv_manipulators_msgs-msg:EndeffectorCommand instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <EndeffectorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-msg:command-val is deprecated.  Use uuv_manipulators_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'ratio-val :lambda-list '(m))
(cl:defmethod ratio-val ((m <EndeffectorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-msg:ratio-val is deprecated.  Use uuv_manipulators_msgs-msg:ratio instead.")
  (ratio m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EndeffectorCommand>)))
    "Constants for message type '<EndeffectorCommand>"
  '((:EE_MOVE . move)
    (:EE_STOP . stop)
    (:EE_CLOSED . 0.0)
    (:EE_OPEN . 100.0))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EndeffectorCommand)))
    "Constants for message type 'EndeffectorCommand"
  '((:EE_MOVE . move)
    (:EE_STOP . stop)
    (:EE_CLOSED . 0.0)
    (:EE_OPEN . 100.0))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EndeffectorCommand>) ostream)
  "Serializes a message object of type '<EndeffectorCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EndeffectorCommand>) istream)
  "Deserializes a message object of type '<EndeffectorCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ratio) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EndeffectorCommand>)))
  "Returns string type for a message object of type '<EndeffectorCommand>"
  "uuv_manipulators_msgs/EndeffectorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndeffectorCommand)))
  "Returns string type for a message object of type 'EndeffectorCommand"
  "uuv_manipulators_msgs/EndeffectorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EndeffectorCommand>)))
  "Returns md5sum for a message object of type '<EndeffectorCommand>"
  "2949dea8a0d479d93952df57c48d98d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EndeffectorCommand)))
  "Returns md5sum for a message object of type 'EndeffectorCommand"
  "2949dea8a0d479d93952df57c48d98d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EndeffectorCommand>)))
  "Returns full string definition for message of type '<EndeffectorCommand>"
  (cl:format cl:nil "# Commands to the end-effector~%string  command     # Operation tag~%# Default commands~%string  EE_MOVE   = move~%string  EE_STOP   = stop~%# Place for arguments, if needed~%float64 ratio~%# Default ratios of aperture~%float64 EE_CLOSED = 0.0~%float64 EE_OPEN   = 100.0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EndeffectorCommand)))
  "Returns full string definition for message of type 'EndeffectorCommand"
  (cl:format cl:nil "# Commands to the end-effector~%string  command     # Operation tag~%# Default commands~%string  EE_MOVE   = move~%string  EE_STOP   = stop~%# Place for arguments, if needed~%float64 ratio~%# Default ratios of aperture~%float64 EE_CLOSED = 0.0~%float64 EE_OPEN   = 100.0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EndeffectorCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EndeffectorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'EndeffectorCommand
    (cl:cons ':command (command msg))
    (cl:cons ':ratio (ratio msg))
))
