; Auto-generated. Do not edit!


(cl:in-package uuv_world_ros_plugins_msgs-srv)


;//! \htmlinclude GetOriginSphericalCoord-request.msg.html

(cl:defclass <GetOriginSphericalCoord-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetOriginSphericalCoord-request (<GetOriginSphericalCoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOriginSphericalCoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOriginSphericalCoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<GetOriginSphericalCoord-request> is deprecated: use uuv_world_ros_plugins_msgs-srv:GetOriginSphericalCoord-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOriginSphericalCoord-request>) ostream)
  "Serializes a message object of type '<GetOriginSphericalCoord-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOriginSphericalCoord-request>) istream)
  "Deserializes a message object of type '<GetOriginSphericalCoord-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOriginSphericalCoord-request>)))
  "Returns string type for a service object of type '<GetOriginSphericalCoord-request>"
  "uuv_world_ros_plugins_msgs/GetOriginSphericalCoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOriginSphericalCoord-request)))
  "Returns string type for a service object of type 'GetOriginSphericalCoord-request"
  "uuv_world_ros_plugins_msgs/GetOriginSphericalCoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOriginSphericalCoord-request>)))
  "Returns md5sum for a message object of type '<GetOriginSphericalCoord-request>"
  "60457d630fe21cc5f8f6bd5d0fc90156")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOriginSphericalCoord-request)))
  "Returns md5sum for a message object of type 'GetOriginSphericalCoord-request"
  "60457d630fe21cc5f8f6bd5d0fc90156")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOriginSphericalCoord-request>)))
  "Returns full string definition for message of type '<GetOriginSphericalCoord-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOriginSphericalCoord-request)))
  "Returns full string definition for message of type 'GetOriginSphericalCoord-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOriginSphericalCoord-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOriginSphericalCoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOriginSphericalCoord-request
))
;//! \htmlinclude GetOriginSphericalCoord-response.msg.html

(cl:defclass <GetOriginSphericalCoord-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetOriginSphericalCoord-response (<GetOriginSphericalCoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOriginSphericalCoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOriginSphericalCoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<GetOriginSphericalCoord-response> is deprecated: use uuv_world_ros_plugins_msgs-srv:GetOriginSphericalCoord-response instead.")))

(cl:ensure-generic-function 'latitude_deg-val :lambda-list '(m))
(cl:defmethod latitude_deg-val ((m <GetOriginSphericalCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:latitude_deg-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:latitude_deg instead.")
  (latitude_deg m))

(cl:ensure-generic-function 'longitude_deg-val :lambda-list '(m))
(cl:defmethod longitude_deg-val ((m <GetOriginSphericalCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:longitude_deg-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:longitude_deg instead.")
  (longitude_deg m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <GetOriginSphericalCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:altitude-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:altitude instead.")
  (altitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOriginSphericalCoord-response>) ostream)
  "Serializes a message object of type '<GetOriginSphericalCoord-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOriginSphericalCoord-response>) istream)
  "Deserializes a message object of type '<GetOriginSphericalCoord-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOriginSphericalCoord-response>)))
  "Returns string type for a service object of type '<GetOriginSphericalCoord-response>"
  "uuv_world_ros_plugins_msgs/GetOriginSphericalCoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOriginSphericalCoord-response)))
  "Returns string type for a service object of type 'GetOriginSphericalCoord-response"
  "uuv_world_ros_plugins_msgs/GetOriginSphericalCoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOriginSphericalCoord-response>)))
  "Returns md5sum for a message object of type '<GetOriginSphericalCoord-response>"
  "60457d630fe21cc5f8f6bd5d0fc90156")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOriginSphericalCoord-response)))
  "Returns md5sum for a message object of type 'GetOriginSphericalCoord-response"
  "60457d630fe21cc5f8f6bd5d0fc90156")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOriginSphericalCoord-response>)))
  "Returns full string definition for message of type '<GetOriginSphericalCoord-response>"
  (cl:format cl:nil "~%float64 latitude_deg~%~%float64 longitude_deg~%~%float64 altitude~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOriginSphericalCoord-response)))
  "Returns full string definition for message of type 'GetOriginSphericalCoord-response"
  (cl:format cl:nil "~%float64 latitude_deg~%~%float64 longitude_deg~%~%float64 altitude~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOriginSphericalCoord-response>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOriginSphericalCoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOriginSphericalCoord-response
    (cl:cons ':latitude_deg (latitude_deg msg))
    (cl:cons ':longitude_deg (longitude_deg msg))
    (cl:cons ':altitude (altitude msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetOriginSphericalCoord)))
  'GetOriginSphericalCoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetOriginSphericalCoord)))
  'GetOriginSphericalCoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOriginSphericalCoord)))
  "Returns string type for a service object of type '<GetOriginSphericalCoord>"
  "uuv_world_ros_plugins_msgs/GetOriginSphericalCoord")