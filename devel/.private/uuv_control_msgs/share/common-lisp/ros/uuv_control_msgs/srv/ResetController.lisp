; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude ResetController-request.msg.html

(cl:defclass <ResetController-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ResetController-request (<ResetController-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetController-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetController-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<ResetController-request> is deprecated: use uuv_control_msgs-srv:ResetController-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetController-request>) ostream)
  "Serializes a message object of type '<ResetController-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetController-request>) istream)
  "Deserializes a message object of type '<ResetController-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetController-request>)))
  "Returns string type for a service object of type '<ResetController-request>"
  "uuv_control_msgs/ResetControllerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetController-request)))
  "Returns string type for a service object of type 'ResetController-request"
  "uuv_control_msgs/ResetControllerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetController-request>)))
  "Returns md5sum for a message object of type '<ResetController-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetController-request)))
  "Returns md5sum for a message object of type 'ResetController-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetController-request>)))
  "Returns full string definition for message of type '<ResetController-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetController-request)))
  "Returns full string definition for message of type 'ResetController-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetController-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetController-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetController-request
))
;//! \htmlinclude ResetController-response.msg.html

(cl:defclass <ResetController-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ResetController-response (<ResetController-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetController-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetController-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<ResetController-response> is deprecated: use uuv_control_msgs-srv:ResetController-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ResetController-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetController-response>) ostream)
  "Serializes a message object of type '<ResetController-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetController-response>) istream)
  "Deserializes a message object of type '<ResetController-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetController-response>)))
  "Returns string type for a service object of type '<ResetController-response>"
  "uuv_control_msgs/ResetControllerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetController-response)))
  "Returns string type for a service object of type 'ResetController-response"
  "uuv_control_msgs/ResetControllerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetController-response>)))
  "Returns md5sum for a message object of type '<ResetController-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetController-response)))
  "Returns md5sum for a message object of type 'ResetController-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetController-response>)))
  "Returns full string definition for message of type '<ResetController-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetController-response)))
  "Returns full string definition for message of type 'ResetController-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetController-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetController-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetController-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetController)))
  'ResetController-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetController)))
  'ResetController-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetController)))
  "Returns string type for a service object of type '<ResetController>"
  "uuv_control_msgs/ResetController")