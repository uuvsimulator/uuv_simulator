; Auto-generated. Do not edit!


(cl:in-package uuv_world_ros_plugins_msgs-srv)


;//! \htmlinclude TransformFromSphericalCoord-request.msg.html

(cl:defclass <TransformFromSphericalCoord-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass TransformFromSphericalCoord-request (<TransformFromSphericalCoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformFromSphericalCoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformFromSphericalCoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<TransformFromSphericalCoord-request> is deprecated: use uuv_world_ros_plugins_msgs-srv:TransformFromSphericalCoord-request instead.")))

(cl:ensure-generic-function 'latitude_deg-val :lambda-list '(m))
(cl:defmethod latitude_deg-val ((m <TransformFromSphericalCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:latitude_deg-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:latitude_deg instead.")
  (latitude_deg m))

(cl:ensure-generic-function 'longitude_deg-val :lambda-list '(m))
(cl:defmethod longitude_deg-val ((m <TransformFromSphericalCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:longitude_deg-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:longitude_deg instead.")
  (longitude_deg m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <TransformFromSphericalCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:altitude-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:altitude instead.")
  (altitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformFromSphericalCoord-request>) ostream)
  "Serializes a message object of type '<TransformFromSphericalCoord-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformFromSphericalCoord-request>) istream)
  "Deserializes a message object of type '<TransformFromSphericalCoord-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformFromSphericalCoord-request>)))
  "Returns string type for a service object of type '<TransformFromSphericalCoord-request>"
  "uuv_world_ros_plugins_msgs/TransformFromSphericalCoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformFromSphericalCoord-request)))
  "Returns string type for a service object of type 'TransformFromSphericalCoord-request"
  "uuv_world_ros_plugins_msgs/TransformFromSphericalCoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformFromSphericalCoord-request>)))
  "Returns md5sum for a message object of type '<TransformFromSphericalCoord-request>"
  "7a7b547d22a150426bbc278358d5fb7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformFromSphericalCoord-request)))
  "Returns md5sum for a message object of type 'TransformFromSphericalCoord-request"
  "7a7b547d22a150426bbc278358d5fb7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformFromSphericalCoord-request>)))
  "Returns full string definition for message of type '<TransformFromSphericalCoord-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 latitude_deg~%~%float64 longitude_deg~%~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformFromSphericalCoord-request)))
  "Returns full string definition for message of type 'TransformFromSphericalCoord-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 latitude_deg~%~%float64 longitude_deg~%~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformFromSphericalCoord-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformFromSphericalCoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformFromSphericalCoord-request
    (cl:cons ':latitude_deg (latitude_deg msg))
    (cl:cons ':longitude_deg (longitude_deg msg))
    (cl:cons ':altitude (altitude msg))
))
;//! \htmlinclude TransformFromSphericalCoord-response.msg.html

(cl:defclass <TransformFromSphericalCoord-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass TransformFromSphericalCoord-response (<TransformFromSphericalCoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformFromSphericalCoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformFromSphericalCoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_world_ros_plugins_msgs-srv:<TransformFromSphericalCoord-response> is deprecated: use uuv_world_ros_plugins_msgs-srv:TransformFromSphericalCoord-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <TransformFromSphericalCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_world_ros_plugins_msgs-srv:output-val is deprecated.  Use uuv_world_ros_plugins_msgs-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformFromSphericalCoord-response>) ostream)
  "Serializes a message object of type '<TransformFromSphericalCoord-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformFromSphericalCoord-response>) istream)
  "Deserializes a message object of type '<TransformFromSphericalCoord-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformFromSphericalCoord-response>)))
  "Returns string type for a service object of type '<TransformFromSphericalCoord-response>"
  "uuv_world_ros_plugins_msgs/TransformFromSphericalCoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformFromSphericalCoord-response)))
  "Returns string type for a service object of type 'TransformFromSphericalCoord-response"
  "uuv_world_ros_plugins_msgs/TransformFromSphericalCoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformFromSphericalCoord-response>)))
  "Returns md5sum for a message object of type '<TransformFromSphericalCoord-response>"
  "7a7b547d22a150426bbc278358d5fb7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformFromSphericalCoord-response)))
  "Returns md5sum for a message object of type 'TransformFromSphericalCoord-response"
  "7a7b547d22a150426bbc278358d5fb7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformFromSphericalCoord-response>)))
  "Returns full string definition for message of type '<TransformFromSphericalCoord-response>"
  (cl:format cl:nil "geometry_msgs/Vector3 output~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformFromSphericalCoord-response)))
  "Returns full string definition for message of type 'TransformFromSphericalCoord-response"
  (cl:format cl:nil "geometry_msgs/Vector3 output~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformFromSphericalCoord-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformFromSphericalCoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformFromSphericalCoord-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TransformFromSphericalCoord)))
  'TransformFromSphericalCoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TransformFromSphericalCoord)))
  'TransformFromSphericalCoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformFromSphericalCoord)))
  "Returns string type for a service object of type '<TransformFromSphericalCoord>"
  "uuv_world_ros_plugins_msgs/TransformFromSphericalCoord")