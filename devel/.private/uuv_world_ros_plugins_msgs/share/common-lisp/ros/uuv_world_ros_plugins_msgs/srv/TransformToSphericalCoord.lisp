; Auto-generated. Do not edit!


(cl:in-package uuv_world_ros_plugins_msgs-srv)


;//! \htmlinclude TransformToSphericalCoord-request.msg.html

(cl:defclass <TransformToSphericalCoord-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass TransformToSphericalCoord-request (<TransformToSphericalCoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformToSphericalCoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformToSphericalCoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<TransformToSphericalCoord-request> is deprecated: use uuv_world_ros_plugins_msgs-srv:TransformToSphericalCoord-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <TransformToSphericalCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:input-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformToSphericalCoord-request>) ostream)
  "Serializes a message object of type '<TransformToSphericalCoord-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformToSphericalCoord-request>) istream)
  "Deserializes a message object of type '<TransformToSphericalCoord-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformToSphericalCoord-request>)))
  "Returns string type for a service object of type '<TransformToSphericalCoord-request>"
  "uuv_world_ros_plugins_msgs/TransformToSphericalCoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformToSphericalCoord-request)))
  "Returns string type for a service object of type 'TransformToSphericalCoord-request"
  "uuv_world_ros_plugins_msgs/TransformToSphericalCoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformToSphericalCoord-request>)))
  "Returns md5sum for a message object of type '<TransformToSphericalCoord-request>"
  "5e63b61b1b56d2a5259cc93964944e7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformToSphericalCoord-request)))
  "Returns md5sum for a message object of type 'TransformToSphericalCoord-request"
  "5e63b61b1b56d2a5259cc93964944e7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformToSphericalCoord-request>)))
  "Returns full string definition for message of type '<TransformToSphericalCoord-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%geometry_msgs/Vector3 input~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformToSphericalCoord-request)))
  "Returns full string definition for message of type 'TransformToSphericalCoord-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%geometry_msgs/Vector3 input~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformToSphericalCoord-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformToSphericalCoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformToSphericalCoord-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude TransformToSphericalCoord-response.msg.html

(cl:defclass <TransformToSphericalCoord-response> (roslisp-msg-protocol:ros-message)
  ((latitude_deg
    :reader latitude_deg
    :initarg :latitude_deg
    :type cl:float
    :initform 0.0)
   (longitude_deg
    :reader longitude_deg
    :initarg :longitude_deg
    :type cl:float
    :initform 0.0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass TransformToSphericalCoord-response (<TransformToSphericalCoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformToSphericalCoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformToSphericalCoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<TransformToSphericalCoord-response> is deprecated: use uuv_world_ros_plugins_msgs-srv:TransformToSphericalCoord-response instead.")))

(cl:ensure-generic-function 'latitude_deg-val :lambda-list '(m))
(cl:defmethod latitude_deg-val ((m <TransformToSphericalCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:latitude_deg-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:latitude_deg instead.")
  (latitude_deg m))

(cl:ensure-generic-function 'longitude_deg-val :lambda-list '(m))
(cl:defmethod longitude_deg-val ((m <TransformToSphericalCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:longitude_deg-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:longitude_deg instead.")
  (longitude_deg m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <TransformToSphericalCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:altitude-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:altitude instead.")
  (altitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformToSphericalCoord-response>) ostream)
  "Serializes a message object of type '<TransformToSphericalCoord-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude_deg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude_deg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformToSphericalCoord-response>) istream)
  "Deserializes a message object of type '<TransformToSphericalCoord-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude_deg) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude_deg) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformToSphericalCoord-response>)))
  "Returns string type for a service object of type '<TransformToSphericalCoord-response>"
  "uuv_world_ros_plugins_msgs/TransformToSphericalCoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformToSphericalCoord-response)))
  "Returns string type for a service object of type 'TransformToSphericalCoord-response"
  "uuv_world_ros_plugins_msgs/TransformToSphericalCoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformToSphericalCoord-response>)))
  "Returns md5sum for a message object of type '<TransformToSphericalCoord-response>"
  "5e63b61b1b56d2a5259cc93964944e7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformToSphericalCoord-response)))
  "Returns md5sum for a message object of type 'TransformToSphericalCoord-response"
  "5e63b61b1b56d2a5259cc93964944e7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformToSphericalCoord-response>)))
  "Returns full string definition for message of type '<TransformToSphericalCoord-response>"
  (cl:format cl:nil "~%float64 latitude_deg~%~%float64 longitude_deg~%~%float64 altitude~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformToSphericalCoord-response)))
  "Returns full string definition for message of type 'TransformToSphericalCoord-response"
  (cl:format cl:nil "~%float64 latitude_deg~%~%float64 longitude_deg~%~%float64 altitude~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformToSphericalCoord-response>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformToSphericalCoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformToSphericalCoord-response
    (cl:cons ':latitude_deg (latitude_deg msg))
    (cl:cons ':longitude_deg (longitude_deg msg))
    (cl:cons ':altitude (altitude msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TransformToSphericalCoord)))
  'TransformToSphericalCoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TransformToSphericalCoord)))
  'TransformToSphericalCoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformToSphericalCoord)))
  "Returns string type for a service object of type '<TransformToSphericalCoord>"
  "uuv_world_ros_plugins_msgs/TransformToSphericalCoord")