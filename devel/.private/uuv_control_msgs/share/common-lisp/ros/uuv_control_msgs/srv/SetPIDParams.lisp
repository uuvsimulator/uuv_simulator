; Auto-generated. Do not edit!


(cl:in-package uuv_control_msgs-srv)


;//! \htmlinclude SetPIDParams-request.msg.html

(cl:defclass <SetPIDParams-request> (roslisp-msg-protocol:ros-message)
  ((Kp
    :reader Kp
    :initarg :Kp
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (Kd
    :reader Kd
    :initarg :Kd
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (Ki
    :reader Ki
    :initarg :Ki
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetPIDParams-request (<SetPIDParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPIDParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPIDParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<SetPIDParams-request> is deprecated: use uuv_control_msgs-srv:SetPIDParams-request instead.")))

(cl:ensure-generic-function 'Kp-val :lambda-list '(m))
(cl:defmethod Kp-val ((m <SetPIDParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:Kp-val is deprecated.  Use uuv_control_msgs-srv:Kp instead.")
  (Kp m))

(cl:ensure-generic-function 'Kd-val :lambda-list '(m))
(cl:defmethod Kd-val ((m <SetPIDParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:Kd-val is deprecated.  Use uuv_control_msgs-srv:Kd instead.")
  (Kd m))

(cl:ensure-generic-function 'Ki-val :lambda-list '(m))
(cl:defmethod Ki-val ((m <SetPIDParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:Ki-val is deprecated.  Use uuv_control_msgs-srv:Ki instead.")
  (Ki m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPIDParams-request>) ostream)
  "Serializes a message object of type '<SetPIDParams-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Kp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'Kp))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Kd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'Kd))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Ki))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'Ki))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPIDParams-request>) istream)
  "Deserializes a message object of type '<SetPIDParams-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Kp) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Kp)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Kd) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Kd)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Ki) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Ki)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPIDParams-request>)))
  "Returns string type for a service object of type '<SetPIDParams-request>"
  "uuv_control_msgs/SetPIDParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPIDParams-request)))
  "Returns string type for a service object of type 'SetPIDParams-request"
  "uuv_control_msgs/SetPIDParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPIDParams-request>)))
  "Returns md5sum for a message object of type '<SetPIDParams-request>"
  "147ee245a68427e8ed98870bce36b399")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPIDParams-request)))
  "Returns md5sum for a message object of type 'SetPIDParams-request"
  "147ee245a68427e8ed98870bce36b399")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPIDParams-request>)))
  "Returns full string definition for message of type '<SetPIDParams-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64[] Kp~%float64[] Kd~%float64[] Ki~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPIDParams-request)))
  "Returns full string definition for message of type 'SetPIDParams-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64[] Kp~%float64[] Kd~%float64[] Ki~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPIDParams-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Kp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Kd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Ki) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPIDParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPIDParams-request
    (cl:cons ':Kp (Kp msg))
    (cl:cons ':Kd (Kd msg))
    (cl:cons ':Ki (Ki msg))
))
;//! \htmlinclude SetPIDParams-response.msg.html

(cl:defclass <SetPIDParams-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetPIDParams-response (<SetPIDParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPIDParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPIDParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_control_msgs-srv:<SetPIDParams-response> is deprecated: use uuv_control_msgs-srv:SetPIDParams-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetPIDParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_control_msgs-srv:success-val is deprecated.  Use uuv_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPIDParams-response>) ostream)
  "Serializes a message object of type '<SetPIDParams-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPIDParams-response>) istream)
  "Deserializes a message object of type '<SetPIDParams-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPIDParams-response>)))
  "Returns string type for a service object of type '<SetPIDParams-response>"
  "uuv_control_msgs/SetPIDParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPIDParams-response)))
  "Returns string type for a service object of type 'SetPIDParams-response"
  "uuv_control_msgs/SetPIDParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPIDParams-response>)))
  "Returns md5sum for a message object of type '<SetPIDParams-response>"
  "147ee245a68427e8ed98870bce36b399")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPIDParams-response)))
  "Returns md5sum for a message object of type 'SetPIDParams-response"
  "147ee245a68427e8ed98870bce36b399")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPIDParams-response>)))
  "Returns full string definition for message of type '<SetPIDParams-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPIDParams-response)))
  "Returns full string definition for message of type 'SetPIDParams-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPIDParams-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPIDParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPIDParams-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPIDParams)))
  'SetPIDParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPIDParams)))
  'SetPIDParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPIDParams)))
  "Returns string type for a service object of type '<SetPIDParams>"
  "uuv_control_msgs/SetPIDParams")