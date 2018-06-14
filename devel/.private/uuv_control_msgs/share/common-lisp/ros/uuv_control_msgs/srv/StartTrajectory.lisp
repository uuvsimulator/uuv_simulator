; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude StartTrajectory-request.msg.html

(cl:defclass <StartTrajectory-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StartTrajectory-request (<StartTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<StartTrajectory-request> is deprecated: use uuv_control_msgs-srv:StartTrajectory-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTrajectory-request>) ostream)
  "Serializes a message object of type '<StartTrajectory-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTrajectory-request>) istream)
  "Deserializes a message object of type '<StartTrajectory-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTrajectory-request>)))
  "Returns string type for a service object of type '<StartTrajectory-request>"
  "uuv_control_msgs/StartTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTrajectory-request)))
  "Returns string type for a service object of type 'StartTrajectory-request"
  "uuv_control_msgs/StartTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTrajectory-request>)))
  "Returns md5sum for a message object of type '<StartTrajectory-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTrajectory-request)))
  "Returns md5sum for a message object of type 'StartTrajectory-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTrajectory-request>)))
  "Returns full string definition for message of type '<StartTrajectory-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTrajectory-request)))
  "Returns full string definition for message of type 'StartTrajectory-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTrajectory-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTrajectory-request
))
;//! \htmlinclude StartTrajectory-response.msg.html

(cl:defclass <StartTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StartTrajectory-response (<StartTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<StartTrajectory-response> is deprecated: use uuv_control_msgs-srv:StartTrajectory-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <StartTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTrajectory-response>) ostream)
  "Serializes a message object of type '<StartTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTrajectory-response>) istream)
  "Deserializes a message object of type '<StartTrajectory-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTrajectory-response>)))
  "Returns string type for a service object of type '<StartTrajectory-response>"
  "uuv_control_msgs/StartTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTrajectory-response)))
  "Returns string type for a service object of type 'StartTrajectory-response"
  "uuv_control_msgs/StartTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTrajectory-response>)))
  "Returns md5sum for a message object of type '<StartTrajectory-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTrajectory-response)))
  "Returns md5sum for a message object of type 'StartTrajectory-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTrajectory-response>)))
  "Returns full string definition for message of type '<StartTrajectory-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTrajectory-response)))
  "Returns full string definition for message of type 'StartTrajectory-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTrajectory-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTrajectory-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartTrajectory)))
  'StartTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartTrajectory)))
  'StartTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTrajectory)))
  "Returns string type for a service object of type '<StartTrajectory>"
  "uuv_control_msgs/StartTrajectory")