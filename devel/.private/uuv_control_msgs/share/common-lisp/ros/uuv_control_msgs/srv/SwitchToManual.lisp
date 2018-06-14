; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude SwitchToManual-request.msg.html

(cl:defclass <SwitchToManual-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SwitchToManual-request (<SwitchToManual-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchToManual-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchToManual-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<SwitchToManual-request> is deprecated: use uuv_control_msgs-srv:SwitchToManual-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchToManual-request>) ostream)
  "Serializes a message object of type '<SwitchToManual-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchToManual-request>) istream)
  "Deserializes a message object of type '<SwitchToManual-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchToManual-request>)))
  "Returns string type for a service object of type '<SwitchToManual-request>"
  "uuv_control_msgs/SwitchToManualRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchToManual-request)))
  "Returns string type for a service object of type 'SwitchToManual-request"
  "uuv_control_msgs/SwitchToManualRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchToManual-request>)))
  "Returns md5sum for a message object of type '<SwitchToManual-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchToManual-request)))
  "Returns md5sum for a message object of type 'SwitchToManual-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchToManual-request>)))
  "Returns full string definition for message of type '<SwitchToManual-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchToManual-request)))
  "Returns full string definition for message of type 'SwitchToManual-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchToManual-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchToManual-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchToManual-request
))
;//! \htmlinclude SwitchToManual-response.msg.html

(cl:defclass <SwitchToManual-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SwitchToManual-response (<SwitchToManual-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchToManual-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchToManual-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<SwitchToManual-response> is deprecated: use uuv_control_msgs-srv:SwitchToManual-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SwitchToManual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchToManual-response>) ostream)
  "Serializes a message object of type '<SwitchToManual-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchToManual-response>) istream)
  "Deserializes a message object of type '<SwitchToManual-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchToManual-response>)))
  "Returns string type for a service object of type '<SwitchToManual-response>"
  "uuv_control_msgs/SwitchToManualResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchToManual-response)))
  "Returns string type for a service object of type 'SwitchToManual-response"
  "uuv_control_msgs/SwitchToManualResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchToManual-response>)))
  "Returns md5sum for a message object of type '<SwitchToManual-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchToManual-response)))
  "Returns md5sum for a message object of type 'SwitchToManual-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchToManual-response>)))
  "Returns full string definition for message of type '<SwitchToManual-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchToManual-response)))
  "Returns full string definition for message of type 'SwitchToManual-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchToManual-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchToManual-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchToManual-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SwitchToManual)))
  'SwitchToManual-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SwitchToManual)))
  'SwitchToManual-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchToManual)))
  "Returns string type for a service object of type '<SwitchToManual>"
  "uuv_control_msgs/SwitchToManual")