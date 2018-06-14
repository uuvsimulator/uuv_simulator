; Auto-generated. Do not edit!


(cl:in-package uuv_sensor_plugins_ros_msgs-srv)


;//! \htmlinclude ChangeSensorState-request.msg.html

(cl:defclass <ChangeSensorState-request> (roslisp-msg-protocol:ros-message)
  ((on
    :reader on
    :initarg :on
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ChangeSensorState-request (<ChangeSensorState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeSensorState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeSensorState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_sensor_plugins_ros_msgs-srv:<ChangeSensorState-request> is deprecated: use uuv_sensor_plugins_ros_msgs-srv:ChangeSensorState-request instead.")))

(cl:ensure-generic-function 'on-val :lambda-list '(m))
(cl:defmethod on-val ((m <ChangeSensorState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-srv:on-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-srv:on instead.")
  (on m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeSensorState-request>) ostream)
  "Serializes a message object of type '<ChangeSensorState-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'on) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeSensorState-request>) istream)
  "Deserializes a message object of type '<ChangeSensorState-request>"
    (cl:setf (cl:slot-value msg 'on) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeSensorState-request>)))
  "Returns string type for a service object of type '<ChangeSensorState-request>"
  "uuv_sensor_plugins_ros_msgs/ChangeSensorStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeSensorState-request)))
  "Returns string type for a service object of type 'ChangeSensorState-request"
  "uuv_sensor_plugins_ros_msgs/ChangeSensorStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeSensorState-request>)))
  "Returns md5sum for a message object of type '<ChangeSensorState-request>"
  "d1c85d57f0ffdd55759c88fc35773fc9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeSensorState-request)))
  "Returns md5sum for a message object of type 'ChangeSensorState-request"
  "d1c85d57f0ffdd55759c88fc35773fc9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeSensorState-request>)))
  "Returns full string definition for message of type '<ChangeSensorState-request>"
  (cl:format cl:nil "bool on~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeSensorState-request)))
  "Returns full string definition for message of type 'ChangeSensorState-request"
  (cl:format cl:nil "bool on~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeSensorState-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeSensorState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeSensorState-request
    (cl:cons ':on (on msg))
))
;//! \htmlinclude ChangeSensorState-response.msg.html

(cl:defclass <ChangeSensorState-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass ChangeSensorState-response (<ChangeSensorState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeSensorState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeSensorState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_sensor_plugins_ros_msgs-srv:<ChangeSensorState-response> is deprecated: use uuv_sensor_plugins_ros_msgs-srv:ChangeSensorState-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ChangeSensorState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-srv:success-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ChangeSensorState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_plugins_ros_msgs-srv:message-val is deprecated.  Use uuv_sensor_plugins_ros_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeSensorState-response>) ostream)
  "Serializes a message object of type '<ChangeSensorState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeSensorState-response>) istream)
  "Deserializes a message object of type '<ChangeSensorState-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeSensorState-response>)))
  "Returns string type for a service object of type '<ChangeSensorState-response>"
  "uuv_sensor_plugins_ros_msgs/ChangeSensorStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeSensorState-response)))
  "Returns string type for a service object of type 'ChangeSensorState-response"
  "uuv_sensor_plugins_ros_msgs/ChangeSensorStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeSensorState-response>)))
  "Returns md5sum for a message object of type '<ChangeSensorState-response>"
  "d1c85d57f0ffdd55759c88fc35773fc9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeSensorState-response)))
  "Returns md5sum for a message object of type 'ChangeSensorState-response"
  "d1c85d57f0ffdd55759c88fc35773fc9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeSensorState-response>)))
  "Returns full string definition for message of type '<ChangeSensorState-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeSensorState-response)))
  "Returns full string definition for message of type 'ChangeSensorState-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeSensorState-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeSensorState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeSensorState-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ChangeSensorState)))
  'ChangeSensorState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ChangeSensorState)))
  'ChangeSensorState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeSensorState)))
  "Returns string type for a service object of type '<ChangeSensorState>"
  "uuv_sensor_plugins_ros_msgs/ChangeSensorState")