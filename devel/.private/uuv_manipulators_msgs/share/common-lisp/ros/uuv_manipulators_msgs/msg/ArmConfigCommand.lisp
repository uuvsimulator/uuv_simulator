; Auto-generated. Do not edit!


(cl:in-package uuv_manipulators_msgs-msg)


;//! \htmlinclude ArmConfigCommand.msg.html

(cl:defclass <ArmConfigCommand> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform "")
   (args
    :reader args
    :initarg :args
    :type cl:string
    :initform ""))
)

(cl:defclass ArmConfigCommand (<ArmConfigCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmConfigCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmConfigCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_manipulators_msgs-msg:<ArmConfigCommand> is deprecated: use uuv_manipulators_msgs-msg:ArmConfigCommand instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <ArmConfigCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-msg:command-val is deprecated.  Use uuv_manipulators_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'args-val :lambda-list '(m))
(cl:defmethod args-val ((m <ArmConfigCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-msg:args-val is deprecated.  Use uuv_manipulators_msgs-msg:args instead.")
  (args m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ArmConfigCommand>)))
    "Constants for message type '<ArmConfigCommand>"
  '((:HOME . home))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ArmConfigCommand)))
    "Constants for message type 'ArmConfigCommand"
  '((:HOME . home))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmConfigCommand>) ostream)
  "Serializes a message object of type '<ArmConfigCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'args))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'args))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmConfigCommand>) istream)
  "Deserializes a message object of type '<ArmConfigCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'args) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'args) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmConfigCommand>)))
  "Returns string type for a message object of type '<ArmConfigCommand>"
  "uuv_manipulators_msgs/ArmConfigCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmConfigCommand)))
  "Returns string type for a message object of type 'ArmConfigCommand"
  "uuv_manipulators_msgs/ArmConfigCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmConfigCommand>)))
  "Returns md5sum for a message object of type '<ArmConfigCommand>"
  "428ecc7602e5c382dfc52081cc34f5a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmConfigCommand)))
  "Returns md5sum for a message object of type 'ArmConfigCommand"
  "428ecc7602e5c382dfc52081cc34f5a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmConfigCommand>)))
  "Returns full string definition for message of type '<ArmConfigCommand>"
  (cl:format cl:nil "# Commands to drive the arm to default configuration~%string  command     # Operation tag~%# Default commands~%string  HOME      = home~%# Place for arguments, if needed~%string  args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmConfigCommand)))
  "Returns full string definition for message of type 'ArmConfigCommand"
  (cl:format cl:nil "# Commands to drive the arm to default configuration~%string  command     # Operation tag~%# Default commands~%string  HOME      = home~%# Place for arguments, if needed~%string  args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmConfigCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
     4 (cl:length (cl:slot-value msg 'args))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmConfigCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmConfigCommand
    (cl:cons ':command (command msg))
    (cl:cons ':args (args msg))
))
