; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude Hold-request.msg.html

(cl:defclass <Hold-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Hold-request (<Hold-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hold-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hold-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<Hold-request> is deprecated: use uuv_control_msgs-srv:Hold-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hold-request>) ostream)
  "Serializes a message object of type '<Hold-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hold-request>) istream)
  "Deserializes a message object of type '<Hold-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hold-request>)))
  "Returns string type for a service object of type '<Hold-request>"
  "uuv_control_msgs/HoldRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hold-request)))
  "Returns string type for a service object of type 'Hold-request"
  "uuv_control_msgs/HoldRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hold-request>)))
  "Returns md5sum for a message object of type '<Hold-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hold-request)))
  "Returns md5sum for a message object of type 'Hold-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hold-request>)))
  "Returns full string definition for message of type '<Hold-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hold-request)))
  "Returns full string definition for message of type 'Hold-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hold-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hold-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Hold-request
))
;//! \htmlinclude Hold-response.msg.html

(cl:defclass <Hold-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Hold-response (<Hold-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hold-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hold-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<Hold-response> is deprecated: use uuv_control_msgs-srv:Hold-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Hold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hold-response>) ostream)
  "Serializes a message object of type '<Hold-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hold-response>) istream)
  "Deserializes a message object of type '<Hold-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hold-response>)))
  "Returns string type for a service object of type '<Hold-response>"
  "uuv_control_msgs/HoldResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hold-response)))
  "Returns string type for a service object of type 'Hold-response"
  "uuv_control_msgs/HoldResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hold-response>)))
  "Returns md5sum for a message object of type '<Hold-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hold-response)))
  "Returns md5sum for a message object of type 'Hold-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hold-response>)))
  "Returns full string definition for message of type '<Hold-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hold-response)))
  "Returns full string definition for message of type 'Hold-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hold-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hold-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Hold-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Hold)))
  'Hold-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Hold)))
  'Hold-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hold)))
  "Returns string type for a service object of type '<Hold>"
  "uuv_control_msgs/Hold")