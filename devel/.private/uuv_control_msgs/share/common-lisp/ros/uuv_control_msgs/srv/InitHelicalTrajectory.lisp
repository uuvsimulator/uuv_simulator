; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude InitHelicalTrajectory-request.msg.html

(cl:defclass <InitHelicalTrajectory-request> (roslisp-msg-protocol:ros-message)
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
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0)
   (center
    :reader center
    :initarg :center
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (is_clockwise
    :reader is_clockwise
    :initarg :is_clockwise
    :type cl:boolean
    :initform cl:nil)
   (angle_offset
    :reader angle_offset
    :initarg :angle_offset
    :type cl:float
    :initform 0.0)
   (n_points
    :reader n_points
    :initarg :n_points
    :type cl:integer
    :initform 0)
   (heading_offset
    :reader heading_offset
    :initarg :heading_offset
    :type cl:float
    :initform 0.0)
   (max_forward_speed
    :reader max_forward_speed
    :initarg :max_forward_speed
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0)
   (n_turns
    :reader n_turns
    :initarg :n_turns
    :type cl:float
    :initform 0.0)
   (delta_z
    :reader delta_z
    :initarg :delta_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass InitHelicalTrajectory-request (<InitHelicalTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitHelicalTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitHelicalTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<InitHelicalTrajectory-request> is deprecated: use uuv_control_msgs-srv:InitHelicalTrajectory-request instead.")))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:start_time-val is deprecated.  Use uuv_control_msgs-srv:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'start_now-val :lambda-list '(m))
(cl:defmethod start_now-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:start_now-val is deprecated.  Use uuv_control_msgs-srv:start_now instead.")
  (start_now m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:radius-val is deprecated.  Use uuv_control_msgs-srv:radius instead.")
  (radius m))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:center-val is deprecated.  Use uuv_control_msgs-srv:center instead.")
  (center m))

(cl:ensure-generic-function 'is_clockwise-val :lambda-list '(m))
(cl:defmethod is_clockwise-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:is_clockwise-val is deprecated.  Use uuv_control_msgs-srv:is_clockwise instead.")
  (is_clockwise m))

(cl:ensure-generic-function 'angle_offset-val :lambda-list '(m))
(cl:defmethod angle_offset-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:angle_offset-val is deprecated.  Use uuv_control_msgs-srv:angle_offset instead.")
  (angle_offset m))

(cl:ensure-generic-function 'n_points-val :lambda-list '(m))
(cl:defmethod n_points-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:n_points-val is deprecated.  Use uuv_control_msgs-srv:n_points instead.")
  (n_points m))

(cl:ensure-generic-function 'heading_offset-val :lambda-list '(m))
(cl:defmethod heading_offset-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:heading_offset-val is deprecated.  Use uuv_control_msgs-srv:heading_offset instead.")
  (heading_offset m))

(cl:ensure-generic-function 'max_forward_speed-val :lambda-list '(m))
(cl:defmethod max_forward_speed-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:max_forward_speed-val is deprecated.  Use uuv_control_msgs-srv:max_forward_speed instead.")
  (max_forward_speed m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:duration-val is deprecated.  Use uuv_control_msgs-srv:duration instead.")
  (duration m))

(cl:ensure-generic-function 'n_turns-val :lambda-list '(m))
(cl:defmethod n_turns-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:n_turns-val is deprecated.  Use uuv_control_msgs-srv:n_turns instead.")
  (n_turns m))

(cl:ensure-generic-function 'delta_z-val :lambda-list '(m))
(cl:defmethod delta_z-val ((m <InitHelicalTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:delta_z-val is deprecated.  Use uuv_control_msgs-srv:delta_z instead.")
  (delta_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitHelicalTrajectory-request>) ostream)
  "Serializes a message object of type '<InitHelicalTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_time) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start_now) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_clockwise) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'n_points)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'n_turns))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitHelicalTrajectory-request>) istream)
  "Deserializes a message object of type '<InitHelicalTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_time) istream)
    (cl:setf (cl:slot-value msg 'start_now) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
    (cl:setf (cl:slot-value msg 'is_clockwise) (cl:not (cl:zerop (cl:read-byte istream))))
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_points) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'n_turns) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitHelicalTrajectory-request>)))
  "Returns string type for a service object of type '<InitHelicalTrajectory-request>"
  "uuv_control_msgs/InitHelicalTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitHelicalTrajectory-request)))
  "Returns string type for a service object of type 'InitHelicalTrajectory-request"
  "uuv_control_msgs/InitHelicalTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitHelicalTrajectory-request>)))
  "Returns md5sum for a message object of type '<InitHelicalTrajectory-request>"
  "bae09a54d9b06eeca193015644eeb493")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitHelicalTrajectory-request)))
  "Returns md5sum for a message object of type 'InitHelicalTrajectory-request"
  "bae09a54d9b06eeca193015644eeb493")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitHelicalTrajectory-request>)))
  "Returns full string definition for message of type '<InitHelicalTrajectory-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%std_msgs/Time start_time~%bool start_now~%float64 radius~%geometry_msgs/Point center~%bool is_clockwise~%float64 angle_offset~%int32 n_points~%float64 heading_offset~%float64 max_forward_speed~%float64 duration~%float64 n_turns~%float64 delta_z~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitHelicalTrajectory-request)))
  "Returns full string definition for message of type 'InitHelicalTrajectory-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%std_msgs/Time start_time~%bool start_now~%float64 radius~%geometry_msgs/Point center~%bool is_clockwise~%float64 angle_offset~%int32 n_points~%float64 heading_offset~%float64 max_forward_speed~%float64 duration~%float64 n_turns~%float64 delta_z~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitHelicalTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_time))
     1
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     1
     8
     4
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitHelicalTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InitHelicalTrajectory-request
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':start_now (start_now msg))
    (cl:cons ':radius (radius msg))
    (cl:cons ':center (center msg))
    (cl:cons ':is_clockwise (is_clockwise msg))
    (cl:cons ':angle_offset (angle_offset msg))
    (cl:cons ':n_points (n_points msg))
    (cl:cons ':heading_offset (heading_offset msg))
    (cl:cons ':max_forward_speed (max_forward_speed msg))
    (cl:cons ':duration (duration msg))
    (cl:cons ':n_turns (n_turns msg))
    (cl:cons ':delta_z (delta_z msg))
))
;//! \htmlinclude InitHelicalTrajectory-response.msg.html

(cl:defclass <InitHelicalTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass InitHelicalTrajectory-response (<InitHelicalTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitHelicalTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitHelicalTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<InitHelicalTrajectory-response> is deprecated: use uuv_control_msgs-srv:InitHelicalTrajectory-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <InitHelicalTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitHelicalTrajectory-response>) ostream)
  "Serializes a message object of type '<InitHelicalTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitHelicalTrajectory-response>) istream)
  "Deserializes a message object of type '<InitHelicalTrajectory-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitHelicalTrajectory-response>)))
  "Returns string type for a service object of type '<InitHelicalTrajectory-response>"
  "uuv_control_msgs/InitHelicalTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitHelicalTrajectory-response)))
  "Returns string type for a service object of type 'InitHelicalTrajectory-response"
  "uuv_control_msgs/InitHelicalTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitHelicalTrajectory-response>)))
  "Returns md5sum for a message object of type '<InitHelicalTrajectory-response>"
  "bae09a54d9b06eeca193015644eeb493")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitHelicalTrajectory-response)))
  "Returns md5sum for a message object of type 'InitHelicalTrajectory-response"
  "bae09a54d9b06eeca193015644eeb493")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitHelicalTrajectory-response>)))
  "Returns full string definition for message of type '<InitHelicalTrajectory-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitHelicalTrajectory-response)))
  "Returns full string definition for message of type 'InitHelicalTrajectory-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitHelicalTrajectory-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitHelicalTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InitHelicalTrajectory-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InitHelicalTrajectory)))
  'InitHelicalTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InitHelicalTrajectory)))
  'InitHelicalTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitHelicalTrajectory)))
  "Returns string type for a service object of type '<InitHelicalTrajectory>"
  "uuv_control_msgs/InitHelicalTrajectory")