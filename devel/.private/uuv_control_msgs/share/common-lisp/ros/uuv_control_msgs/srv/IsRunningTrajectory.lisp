; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude IsRunningTrajectory-request.msg.html

(cl:defclass <IsRunningTrajectory-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass IsRunningTrajectory-request (<IsRunningTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsRunningTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsRunningTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<IsRunningTrajectory-request> is deprecated: use uuv_control_msgs-srv:IsRunningTrajectory-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsRunningTrajectory-request>) ostream)
  "Serializes a message object of type '<IsRunningTrajectory-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsRunningTrajectory-request>) istream)
  "Deserializes a message object of type '<IsRunningTrajectory-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsRunningTrajectory-request>)))
  "Returns string type for a service object of type '<IsRunningTrajectory-request>"
  "uuv_control_msgs/IsRunningTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsRunningTrajectory-request)))
  "Returns string type for a service object of type 'IsRunningTrajectory-request"
  "uuv_control_msgs/IsRunningTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsRunningTrajectory-request>)))
  "Returns md5sum for a message object of type '<IsRunningTrajectory-request>"
  "7afca0099e0cddc25243b1e3569895fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsRunningTrajectory-request)))
  "Returns md5sum for a message object of type 'IsRunningTrajectory-request"
  "7afca0099e0cddc25243b1e3569895fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsRunningTrajectory-request>)))
  "Returns full string definition for message of type '<IsRunningTrajectory-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsRunningTrajectory-request)))
  "Returns full string definition for message of type 'IsRunningTrajectory-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsRunningTrajectory-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsRunningTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IsRunningTrajectory-request
))
;//! \htmlinclude IsRunningTrajectory-response.msg.html

(cl:defclass <IsRunningTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((progress
    :reader progress
    :initarg :progress
    :type cl:float
    :initform 0.0))
)

(cl:defclass IsRunningTrajectory-response (<IsRunningTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsRunningTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsRunningTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<IsRunningTrajectory-response> is deprecated: use uuv_control_msgs-srv:IsRunningTrajectory-response instead.")))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <IsRunningTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:progress-val is deprecated.  Use uuv_control_msgs-srv:progress instead.")
  (progress m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsRunningTrajectory-response>) ostream)
  "Serializes a message object of type '<IsRunningTrajectory-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'progress))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsRunningTrajectory-response>) istream)
  "Deserializes a message object of type '<IsRunningTrajectory-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'progress) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsRunningTrajectory-response>)))
  "Returns string type for a service object of type '<IsRunningTrajectory-response>"
  "uuv_control_msgs/IsRunningTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsRunningTrajectory-response)))
  "Returns string type for a service object of type 'IsRunningTrajectory-response"
  "uuv_control_msgs/IsRunningTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsRunningTrajectory-response>)))
  "Returns md5sum for a message object of type '<IsRunningTrajectory-response>"
  "7afca0099e0cddc25243b1e3569895fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsRunningTrajectory-response)))
  "Returns md5sum for a message object of type 'IsRunningTrajectory-response"
  "7afca0099e0cddc25243b1e3569895fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsRunningTrajectory-response>)))
  "Returns full string definition for message of type '<IsRunningTrajectory-response>"
  (cl:format cl:nil "float64 progress~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsRunningTrajectory-response)))
  "Returns full string definition for message of type 'IsRunningTrajectory-response"
  (cl:format cl:nil "float64 progress~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsRunningTrajectory-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsRunningTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IsRunningTrajectory-response
    (cl:cons ':progress (progress msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IsRunningTrajectory)))
  'IsRunningTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IsRunningTrajectory)))
  'IsRunningTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsRunningTrajectory)))
  "Returns string type for a service object of type '<IsRunningTrajectory>"
  "uuv_control_msgs/IsRunningTrajectory")