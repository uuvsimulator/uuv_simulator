; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude SwitchToAutomatic-request.msg.html

(cl:defclass <SwitchToAutomatic-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SwitchToAutomatic-request (<SwitchToAutomatic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchToAutomatic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchToAutomatic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<SwitchToAutomatic-request> is deprecated: use uuv_control_msgs-srv:SwitchToAutomatic-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchToAutomatic-request>) ostream)
  "Serializes a message object of type '<SwitchToAutomatic-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchToAutomatic-request>) istream)
  "Deserializes a message object of type '<SwitchToAutomatic-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchToAutomatic-request>)))
  "Returns string type for a service object of type '<SwitchToAutomatic-request>"
  "uuv_control_msgs/SwitchToAutomaticRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchToAutomatic-request)))
  "Returns string type for a service object of type 'SwitchToAutomatic-request"
  "uuv_control_msgs/SwitchToAutomaticRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchToAutomatic-request>)))
  "Returns md5sum for a message object of type '<SwitchToAutomatic-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchToAutomatic-request)))
  "Returns md5sum for a message object of type 'SwitchToAutomatic-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchToAutomatic-request>)))
  "Returns full string definition for message of type '<SwitchToAutomatic-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchToAutomatic-request)))
  "Returns full string definition for message of type 'SwitchToAutomatic-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchToAutomatic-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchToAutomatic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchToAutomatic-request
))
;//! \htmlinclude SwitchToAutomatic-response.msg.html

(cl:defclass <SwitchToAutomatic-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SwitchToAutomatic-response (<SwitchToAutomatic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchToAutomatic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchToAutomatic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<SwitchToAutomatic-response> is deprecated: use uuv_control_msgs-srv:SwitchToAutomatic-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SwitchToAutomatic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchToAutomatic-response>) ostream)
  "Serializes a message object of type '<SwitchToAutomatic-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchToAutomatic-response>) istream)
  "Deserializes a message object of type '<SwitchToAutomatic-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchToAutomatic-response>)))
  "Returns string type for a service object of type '<SwitchToAutomatic-response>"
  "uuv_control_msgs/SwitchToAutomaticResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchToAutomatic-response)))
  "Returns string type for a service object of type 'SwitchToAutomatic-response"
  "uuv_control_msgs/SwitchToAutomaticResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchToAutomatic-response>)))
  "Returns md5sum for a message object of type '<SwitchToAutomatic-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchToAutomatic-response)))
  "Returns md5sum for a message object of type 'SwitchToAutomatic-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchToAutomatic-response>)))
  "Returns full string definition for message of type '<SwitchToAutomatic-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchToAutomatic-response)))
  "Returns full string definition for message of type 'SwitchToAutomatic-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchToAutomatic-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchToAutomatic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchToAutomatic-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SwitchToAutomatic)))
  'SwitchToAutomatic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SwitchToAutomatic)))
  'SwitchToAutomatic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchToAutomatic)))
  "Returns string type for a service object of type '<SwitchToAutomatic>"
  "uuv_control_msgs/SwitchToAutomatic")