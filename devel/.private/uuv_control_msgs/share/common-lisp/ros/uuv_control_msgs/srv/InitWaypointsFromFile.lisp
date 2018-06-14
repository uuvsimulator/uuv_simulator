; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude InitWaypointsFromFile-request.msg.html

(cl:defclass <InitWaypointsFromFile-request> (roslisp-msg-protocol:ros-message)
  ((start_time
    :reader start_time
    :initarg :start_time
    :type std_msgs-msg:Time
    :initform (cl:make-instance 'std_msgs-msg:Time))
   (start_now
    :reader start_now
    :initarg :start_now
    :type cl:boolean
    :initform cl:nil)
   (filename
    :reader filename
    :initarg :filename
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (interpolator
    :reader interpolator
    :initarg :interpolator
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass InitWaypointsFromFile-request (<InitWaypointsFromFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitWaypointsFromFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitWaypointsFromFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<InitWaypointsFromFile-request> is deprecated: use uuv_control_msgs-srv:InitWaypointsFromFile-request instead.")))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <InitWaypointsFromFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:start_time-val is deprecated.  Use uuv_control_msgs-srv:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'start_now-val :lambda-list '(m))
(cl:defmethod start_now-val ((m <InitWaypointsFromFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:start_now-val is deprecated.  Use uuv_control_msgs-srv:start_now instead.")
  (start_now m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <InitWaypointsFromFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:filename-val is deprecated.  Use uuv_control_msgs-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'interpolator-val :lambda-list '(m))
(cl:defmethod interpolator-val ((m <InitWaypointsFromFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:interpolator-val is deprecated.  Use uuv_control_msgs-srv:interpolator instead.")
  (interpolator m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<InitWaypointsFromFile-request>)))
    "Constants for message type '<InitWaypointsFromFile-request>"
  '((:LIPB . 'lipb')
    (:CUBIC . 'cubic')
    (:DUBINS . 'dubins')
    (:LINEAR . 'linear'))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'InitWaypointsFromFile-request)))
    "Constants for message type 'InitWaypointsFromFile-request"
  '((:LIPB . 'lipb')
    (:CUBIC . 'cubic')
    (:DUBINS . 'dubins')
    (:LINEAR . 'linear'))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitWaypointsFromFile-request>) ostream)
  "Serializes a message object of type '<InitWaypointsFromFile-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_time) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start_now) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'filename) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'interpolator) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitWaypointsFromFile-request>) istream)
  "Deserializes a message object of type '<InitWaypointsFromFile-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_time) istream)
    (cl:setf (cl:slot-value msg 'start_now) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'filename) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'interpolator) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitWaypointsFromFile-request>)))
  "Returns string type for a service object of type '<InitWaypointsFromFile-request>"
  "uuv_control_msgs/InitWaypointsFromFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitWaypointsFromFile-request)))
  "Returns string type for a service object of type 'InitWaypointsFromFile-request"
  "uuv_control_msgs/InitWaypointsFromFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitWaypointsFromFile-request>)))
  "Returns md5sum for a message object of type '<InitWaypointsFromFile-request>"
  "a0198a3f7b74ab5dc081e88db6d85c58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitWaypointsFromFile-request)))
  "Returns md5sum for a message object of type 'InitWaypointsFromFile-request"
  "a0198a3f7b74ab5dc081e88db6d85c58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitWaypointsFromFile-request>)))
  "Returns full string definition for message of type '<InitWaypointsFromFile-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string LIPB='lipb'~%string CUBIC='cubic'~%string DUBINS='dubins'~%string LINEAR='linear'~%~%std_msgs/Time start_time~%bool start_now~%std_msgs/String filename~%std_msgs/String interpolator~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitWaypointsFromFile-request)))
  "Returns full string definition for message of type 'InitWaypointsFromFile-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string LIPB='lipb'~%string CUBIC='cubic'~%string DUBINS='dubins'~%string LINEAR='linear'~%~%std_msgs/Time start_time~%bool start_now~%std_msgs/String filename~%std_msgs/String interpolator~%~%================================================================================~%MSG: std_msgs/Time~%time data~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitWaypointsFromFile-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_time))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'filename))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'interpolator))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitWaypointsFromFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InitWaypointsFromFile-request
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':start_now (start_now msg))
    (cl:cons ':filename (filename msg))
    (cl:cons ':interpolator (interpolator msg))
))
;//! \htmlinclude InitWaypointsFromFile-response.msg.html

(cl:defclass <InitWaypointsFromFile-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass InitWaypointsFromFile-response (<InitWaypointsFromFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InitWaypointsFromFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InitWaypointsFromFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<InitWaypointsFromFile-response> is deprecated: use uuv_control_msgs-srv:InitWaypointsFromFile-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <InitWaypointsFromFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InitWaypointsFromFile-response>) ostream)
  "Serializes a message object of type '<InitWaypointsFromFile-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InitWaypointsFromFile-response>) istream)
  "Deserializes a message object of type '<InitWaypointsFromFile-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InitWaypointsFromFile-response>)))
  "Returns string type for a service object of type '<InitWaypointsFromFile-response>"
  "uuv_control_msgs/InitWaypointsFromFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitWaypointsFromFile-response)))
  "Returns string type for a service object of type 'InitWaypointsFromFile-response"
  "uuv_control_msgs/InitWaypointsFromFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InitWaypointsFromFile-response>)))
  "Returns md5sum for a message object of type '<InitWaypointsFromFile-response>"
  "a0198a3f7b74ab5dc081e88db6d85c58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InitWaypointsFromFile-response)))
  "Returns md5sum for a message object of type 'InitWaypointsFromFile-response"
  "a0198a3f7b74ab5dc081e88db6d85c58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InitWaypointsFromFile-response>)))
  "Returns full string definition for message of type '<InitWaypointsFromFile-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InitWaypointsFromFile-response)))
  "Returns full string definition for message of type 'InitWaypointsFromFile-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InitWaypointsFromFile-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InitWaypointsFromFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InitWaypointsFromFile-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InitWaypointsFromFile)))
  'InitWaypointsFromFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InitWaypointsFromFile)))
  'InitWaypointsFromFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InitWaypointsFromFile)))
  "Returns string type for a service object of type '<InitWaypointsFromFile>"
  "uuv_control_msgs/InitWaypointsFromFile")