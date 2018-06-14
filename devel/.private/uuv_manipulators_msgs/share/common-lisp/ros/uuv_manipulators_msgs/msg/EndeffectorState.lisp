; Auto-generated. Do not edit!


(cl:in-package uuv_manipulators_msgs-msg)


;//! \htmlinclude EndeffectorState.msg.html

(cl:defclass <EndeffectorState> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0)
   (effort
    :reader effort
    :initarg :effort
    :type cl:float
    :initform 0.0))
)

(cl:defclass EndeffectorState (<EndeffectorState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EndeffectorState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EndeffectorState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_manipulators_msgs-msg:<EndeffectorState> is deprecated: use uuv_manipulators_msgs-msg:EndeffectorState instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <EndeffectorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-msg:stamp-val is deprecated.  Use uuv_manipulators_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <EndeffectorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-msg:state-val is deprecated.  Use uuv_manipulators_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <EndeffectorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-msg:position-val is deprecated.  Use uuv_manipulators_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'effort-val :lambda-list '(m))
(cl:defmethod effort-val ((m <EndeffectorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-msg:effort-val is deprecated.  Use uuv_manipulators_msgs-msg:effort instead.")
  (effort m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EndeffectorState>)))
    "Constants for message type '<EndeffectorState>"
  '((:MOVING . moving)
    (:DISABLED . disabled)
    (:READY . ready))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EndeffectorState)))
    "Constants for message type 'EndeffectorState"
  '((:MOVING . moving)
    (:DISABLED . disabled)
    (:READY . ready))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EndeffectorState>) ostream)
  "Serializes a message object of type '<EndeffectorState>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'effort))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EndeffectorState>) istream)
  "Deserializes a message object of type '<EndeffectorState>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'effort) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EndeffectorState>)))
  "Returns string type for a message object of type '<EndeffectorState>"
  "uuv_manipulators_msgs/EndeffectorState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndeffectorState)))
  "Returns string type for a message object of type 'EndeffectorState"
  "uuv_manipulators_msgs/EndeffectorState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EndeffectorState>)))
  "Returns md5sum for a message object of type '<EndeffectorState>"
  "696be13165827a929d588e22744f2679")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EndeffectorState)))
  "Returns md5sum for a message object of type 'EndeffectorState"
  "696be13165827a929d588e22744f2679")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EndeffectorState>)))
  "Returns full string definition for message of type '<EndeffectorState>"
  (cl:format cl:nil "# States of the end-effector~%time      stamp~%string    state~%# Default states~%string    MOVING      = moving~%string    DISABLED    = disabled~%string    READY       = ready~%# Current state~%float64   position~%float64   effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EndeffectorState)))
  "Returns full string definition for message of type 'EndeffectorState"
  (cl:format cl:nil "# States of the end-effector~%time      stamp~%string    state~%# Default states~%string    MOVING      = moving~%string    DISABLED    = disabled~%string    READY       = ready~%# Current state~%float64   position~%float64   effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EndeffectorState>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'state))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EndeffectorState>))
  "Converts a ROS message object to a list"
  (cl:list 'EndeffectorState
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':state (state msg))
    (cl:cons ':position (position msg))
    (cl:cons ':effort (effort msg))
))
