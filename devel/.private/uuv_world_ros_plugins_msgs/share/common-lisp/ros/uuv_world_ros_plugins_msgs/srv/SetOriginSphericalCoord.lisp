; Auto-generated. Do not edit!


(cl:in-package uuv_world_ros_plugins_msgs-srv)


;//! \htmlinclude SetOriginSphericalCoord-request.msg.html

(cl:defclass <SetOriginSphericalCoord-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetOriginSphericalCoord-request (<SetOriginSphericalCoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOriginSphericalCoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOriginSphericalCoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<SetOriginSphericalCoord-request> is deprecated: use uuv_world_ros_plugins_msgs-srv:SetOriginSphericalCoord-request instead.")))

(cl:ensure-generic-function 'latitude_deg-val :lambda-list '(m))
(cl:defmethod latitude_deg-val ((m <SetOriginSphericalCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:latitude_deg-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:latitude_deg instead.")
  (latitude_deg m))

(cl:ensure-generic-function 'longitude_deg-val :lambda-list '(m))
(cl:defmethod longitude_deg-val ((m <SetOriginSphericalCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:longitude_deg-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:longitude_deg instead.")
  (longitude_deg m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <SetOriginSphericalCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:altitude-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:altitude instead.")
  (altitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOriginSphericalCoord-request>) ostream)
  "Serializes a message object of type '<SetOriginSphericalCoord-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOriginSphericalCoord-request>) istream)
  "Deserializes a message object of type '<SetOriginSphericalCoord-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOriginSphericalCoord-request>)))
  "Returns string type for a service object of type '<SetOriginSphericalCoord-request>"
  "uuv_world_ros_plugins_msgs/SetOriginSphericalCoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOriginSphericalCoord-request)))
  "Returns string type for a service object of type 'SetOriginSphericalCoord-request"
  "uuv_world_ros_plugins_msgs/SetOriginSphericalCoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOriginSphericalCoord-request>)))
  "Returns md5sum for a message object of type '<SetOriginSphericalCoord-request>"
  "be1cd7093c79a14933c2ac116d54917a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOriginSphericalCoord-request)))
  "Returns md5sum for a message object of type 'SetOriginSphericalCoord-request"
  "be1cd7093c79a14933c2ac116d54917a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOriginSphericalCoord-request>)))
  "Returns full string definition for message of type '<SetOriginSphericalCoord-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 latitude_deg~%~%float64 longitude_deg~%~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOriginSphericalCoord-request)))
  "Returns full string definition for message of type 'SetOriginSphericalCoord-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 latitude_deg~%~%float64 longitude_deg~%~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOriginSphericalCoord-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOriginSphericalCoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOriginSphericalCoord-request
    (cl:cons ':latitude_deg (latitude_deg msg))
    (cl:cons ':longitude_deg (longitude_deg msg))
    (cl:cons ':altitude (altitude msg))
))
;//! \htmlinclude SetOriginSphericalCoord-response.msg.html

(cl:defclass <SetOriginSphericalCoord-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetOriginSphericalCoord-response (<SetOriginSphericalCoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOriginSphericalCoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOriginSphericalCoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<SetOriginSphericalCoord-response> is deprecated: use uuv_world_ros_plugins_msgs-srv:SetOriginSphericalCoord-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetOriginSphericalCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:success-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOriginSphericalCoord-response>) ostream)
  "Serializes a message object of type '<SetOriginSphericalCoord-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOriginSphericalCoord-response>) istream)
  "Deserializes a message object of type '<SetOriginSphericalCoord-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOriginSphericalCoord-response>)))
  "Returns string type for a service object of type '<SetOriginSphericalCoord-response>"
  "uuv_world_ros_plugins_msgs/SetOriginSphericalCoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOriginSphericalCoord-response)))
  "Returns string type for a service object of type 'SetOriginSphericalCoord-response"
  "uuv_world_ros_plugins_msgs/SetOriginSphericalCoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOriginSphericalCoord-response>)))
  "Returns md5sum for a message object of type '<SetOriginSphericalCoord-response>"
  "be1cd7093c79a14933c2ac116d54917a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOriginSphericalCoord-response)))
  "Returns md5sum for a message object of type 'SetOriginSphericalCoord-response"
  "be1cd7093c79a14933c2ac116d54917a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOriginSphericalCoord-response>)))
  "Returns full string definition for message of type '<SetOriginSphericalCoord-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOriginSphericalCoord-response)))
  "Returns full string definition for message of type 'SetOriginSphericalCoord-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOriginSphericalCoord-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOriginSphericalCoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOriginSphericalCoord-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetOriginSphericalCoord)))
  'SetOriginSphericalCoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetOriginSphericalCoord)))
  'SetOriginSphericalCoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOriginSphericalCoord)))
  "Returns string type for a service object of type '<SetOriginSphericalCoord>"
  "uuv_world_ros_plugins_msgs/SetOriginSphericalCoord")