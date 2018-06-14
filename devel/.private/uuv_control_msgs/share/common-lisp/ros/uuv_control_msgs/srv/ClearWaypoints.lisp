; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude ClearWaypoints-request.msg.html

(cl:defclass <ClearWaypoints-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ClearWaypoints-request (<ClearWaypoints-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClearWaypoints-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClearWaypoints-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<ClearWaypoints-request> is deprecated: use uuv_control_msgs-srv:ClearWaypoints-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClearWaypoints-request>) ostream)
  "Serializes a message object of type '<ClearWaypoints-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClearWaypoints-request>) istream)
  "Deserializes a message object of type '<ClearWaypoints-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClearWaypoints-request>)))
  "Returns string type for a service object of type '<ClearWaypoints-request>"
  "uuv_control_msgs/ClearWaypointsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClearWaypoints-request)))
  "Returns string type for a service object of type 'ClearWaypoints-request"
  "uuv_control_msgs/ClearWaypointsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClearWaypoints-request>)))
  "Returns md5sum for a message object of type '<ClearWaypoints-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClearWaypoints-request)))
  "Returns md5sum for a message object of type 'ClearWaypoints-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClearWaypoints-request>)))
  "Returns full string definition for message of type '<ClearWaypoints-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClearWaypoints-request)))
  "Returns full string definition for message of type 'ClearWaypoints-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClearWaypoints-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClearWaypoints-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ClearWaypoints-request
))
;//! \htmlinclude ClearWaypoints-response.msg.html

(cl:defclass <ClearWaypoints-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ClearWaypoints-response (<ClearWaypoints-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClearWaypoints-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClearWaypoints-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<ClearWaypoints-response> is deprecated: use uuv_control_msgs-srv:ClearWaypoints-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ClearWaypoints-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClearWaypoints-response>) ostream)
  "Serializes a message object of type '<ClearWaypoints-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClearWaypoints-response>) istream)
  "Deserializes a message object of type '<ClearWaypoints-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClearWaypoints-response>)))
  "Returns string type for a service object of type '<ClearWaypoints-response>"
  "uuv_control_msgs/ClearWaypointsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClearWaypoints-response)))
  "Returns string type for a service object of type 'ClearWaypoints-response"
  "uuv_control_msgs/ClearWaypointsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClearWaypoints-response>)))
  "Returns md5sum for a message object of type '<ClearWaypoints-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClearWaypoints-response)))
  "Returns md5sum for a message object of type 'ClearWaypoints-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClearWaypoints-response>)))
  "Returns full string definition for message of type '<ClearWaypoints-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClearWaypoints-response)))
  "Returns full string definition for message of type 'ClearWaypoints-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClearWaypoints-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClearWaypoints-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ClearWaypoints-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ClearWaypoints)))
  'ClearWaypoints-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ClearWaypoints)))
  'ClearWaypoints-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClearWaypoints)))
  "Returns string type for a service object of type '<ClearWaypoints>"
  "uuv_control_msgs/ClearWaypoints")