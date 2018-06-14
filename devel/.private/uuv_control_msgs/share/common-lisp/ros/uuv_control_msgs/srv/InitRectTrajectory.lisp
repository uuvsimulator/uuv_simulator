; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude InitRectTrajectory-request.msg.html

(cl:defclass <InitRectTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((start_time
    :reader start_time
    :initarg :start_time
    :type std_msgs-msg:Time
    :initform (cl:make-instance 'std_msgs-msg:Time))
   (start_now
    :reader start_now
    :initarg :start_now
    :type cl:boolean
    :initform cl:nil)
   (origin
    :reader origin
    :initarg :origin
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0)
   (width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0)
   (angle_offset
    :reader angle_offset
    :initarg :angle_offset
    :type cl:float
    :initform 0.0)
   (heading_offset
    :reader heading_offset
    :initarg :heading_offset
    :type cl:float
    :initform 0.0)
   (max_forward_speed
    :reader max_forward_speed
    :initarg :max_forward_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass InitRectTrajectory-request (<InitRectTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitRectTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitRectTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<InitRectTrajectory-request> is deprecated: use uuv_control_msgs-srv:InitRectTrajectory-request instead.")))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <InitRectTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:start_time-val is deprecated.  Use uuv_control_msgs-srv:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'start_now-val :lambda-list '(m))
(cl:defmethod start_now-val ((m <InitRectTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:start_now-val is deprecated.  Use uuv_control_msgs-srv:start_now instead.")
  (start_now m))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <InitRectTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:origin-val is deprecated.  Use uuv_control_msgs-srv:origin instead.")
  (origin m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <InitRectTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:height-val is deprecated.  Use uuv_control_msgs-srv:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <InitRectTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:width-val is deprecated.  Use uuv_control_msgs-srv:width instead.")
  (width m))

(cl:ensure-generic-function 'angle_offset-val :lambda-list '(m))
(cl:defmethod angle_offset-val ((m <InitRectTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:angle_offset-val is deprecated.  Use uuv_control_msgs-srv:angle_offset instead.")
  (angle_offset m))

(cl:ensure-generic-function 'heading_offset-val :lambda-list '(m))
(cl:defmethod heading_offset-val ((m <InitRectTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:heading_offset-val is deprecated.  Use uuv_control_msgs-srv:heading_offset instead.")
  (heading_offset m))

(cl:ensure-generic-function 'max_forward_speed-val :lambda-list '(m))
(cl:defmethod max_forward_speed-val ((m <InitRectTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:max_forward_speed-val is deprecated.  Use uuv_control_msgs-srv:max_forward_speed instead.")
  (max_forward_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitRectTrajectory-request>) ostream)
  "Serializes a message object of type '<InitRectTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_time) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start_now) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_forward_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitRectTrajectory-request>) istream)
  "Deserializes a message object of type '<InitRectTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_time) istream)
    (cl:setf (cl:slot-value msg 'start_now) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_offset) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_offset) (roslisp-utils:decode-double-float-bits bits)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitRectTrajectory-request>)))
  "Returns string type for a service object of type '<InitRectTrajectory-request>"
  "uuv_control_msgs/InitRectTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitRectTrajectory-request)))
  "Returns string type for a service object of type 'InitRectTrajectory-request"
  "uuv_control_msgs/InitRectTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitRectTrajectory-request>)))
  "Returns md5sum for a message object of type '<InitRectTrajectory-request>"
  "bb6b6b97f153ba237ef24a0678facef1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitRectTrajectory-request)))
  "Returns md5sum for a message object of type 'InitRectTrajectory-request"
  "bb6b6b97f153ba237ef24a0678facef1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitRectTrajectory-request>)))
  "Returns full string definition for message of type '<InitRectTrajectory-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%std_msgs/Time start_time~%bool start_now~%geometry_msgs/Point origin~%float64 height~%float64 width~%float64 angle_offset~%float64 heading_offset~%float64 max_forward_speed~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitRectTrajectory-request)))
  "Returns full string definition for message of type 'InitRectTrajectory-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%std_msgs/Time start_time~%bool start_now~%geometry_msgs/Point origin~%float64 height~%float64 width~%float64 angle_offset~%float64 heading_offset~%float64 max_forward_speed~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitRectTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_time))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin))
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitRectTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InitRectTrajectory-request
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':start_now (start_now msg))
    (cl:cons ':origin (origin msg))
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':angle_offset (angle_offset msg))
    (cl:cons ':heading_offset (heading_offset msg))
    (cl:cons ':max_forward_speed (max_forward_speed msg))
))
;//! \htmlinclude InitRectTrajectory-response.msg.html

(cl:defclass <InitRectTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass InitRectTrajectory-response (<InitRectTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitRectTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitRectTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<InitRectTrajectory-response> is deprecated: use uuv_control_msgs-srv:InitRectTrajectory-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <InitRectTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitRectTrajectory-response>) ostream)
  "Serializes a message object of type '<InitRectTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitRectTrajectory-response>) istream)
  "Deserializes a message object of type '<InitRectTrajectory-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitRectTrajectory-response>)))
  "Returns string type for a service object of type '<InitRectTrajectory-response>"
  "uuv_control_msgs/InitRectTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitRectTrajectory-response)))
  "Returns string type for a service object of type 'InitRectTrajectory-response"
  "uuv_control_msgs/InitRectTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitRectTrajectory-response>)))
  "Returns md5sum for a message object of type '<InitRectTrajectory-response>"
  "bb6b6b97f153ba237ef24a0678facef1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitRectTrajectory-response)))
  "Returns md5sum for a message object of type 'InitRectTrajectory-response"
  "bb6b6b97f153ba237ef24a0678facef1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitRectTrajectory-response>)))
  "Returns full string definition for message of type '<InitRectTrajectory-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitRectTrajectory-response)))
  "Returns full string definition for message of type 'InitRectTrajectory-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitRectTrajectory-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitRectTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InitRectTrajectory-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InitRectTrajectory)))
  'InitRectTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InitRectTrajectory)))
  'InitRectTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitRectTrajectory)))
  "Returns string type for a service object of type '<InitRectTrajectory>"
  "uuv_control_msgs/InitRectTrajectory")