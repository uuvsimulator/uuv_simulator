; Auto-generated. Do not edit!


(cl:in-package uuv_thruster_manager-srv)


;//! \htmlinclude GetThrusterCurve-request.msg.html

(cl:defclass <GetThrusterCurve-request> (roslisp-msg-protocol:ros-message)
  ((min
    :reader min
    :initarg :min
    :type cl:float
    :initform 0.0)
   (max
    :reader max
    :initarg :max
    :type cl:float
    :initform 0.0)
   (n_points
    :reader n_points
    :initarg :n_points
    :type cl:integer
    :initform 0))
)

(cl:defclass GetThrusterCurve-request (<GetThrusterCurve-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetThrusterCurve-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetThrusterCurve-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_thruster_manager-srv:<GetThrusterCurve-request> is deprecated: use uuv_thruster_manager-srv:GetThrusterCurve-request instead.")))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <GetThrusterCurve-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:min-val is deprecated.  Use uuv_thruster_manager-srv:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <GetThrusterCurve-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:max-val is deprecated.  Use uuv_thruster_manager-srv:max instead.")
  (max m))

(cl:ensure-generic-function 'n_points-val :lambda-list '(m))
(cl:defmethod n_points-val ((m <GetThrusterCurve-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:n_points-val is deprecated.  Use uuv_thruster_manager-srv:n_points instead.")
  (n_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetThrusterCurve-request>) ostream)
  "Serializes a message object of type '<GetThrusterCurve-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'n_points)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetThrusterCurve-request>) istream)
  "Deserializes a message object of type '<GetThrusterCurve-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_points) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetThrusterCurve-request>)))
  "Returns string type for a service object of type '<GetThrusterCurve-request>"
  "uuv_thruster_manager/GetThrusterCurveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterCurve-request)))
  "Returns string type for a service object of type 'GetThrusterCurve-request"
  "uuv_thruster_manager/GetThrusterCurveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetThrusterCurve-request>)))
  "Returns md5sum for a message object of type '<GetThrusterCurve-request>"
  "93e0d8a78977b3619bdc09f290ef57fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetThrusterCurve-request)))
  "Returns md5sum for a message object of type 'GetThrusterCurve-request"
  "93e0d8a78977b3619bdc09f290ef57fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetThrusterCurve-request>)))
  "Returns full string definition for message of type '<GetThrusterCurve-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 min~%float64 max~%int32 n_points~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetThrusterCurve-request)))
  "Returns full string definition for message of type 'GetThrusterCurve-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%float64 min~%float64 max~%int32 n_points~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetThrusterCurve-request>))
  (cl:+ 0
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetThrusterCurve-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetThrusterCurve-request
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
    (cl:cons ':n_points (n_points msg))
))
;//! \htmlinclude GetThrusterCurve-response.msg.html

(cl:defclass <GetThrusterCurve-response> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (thrust
    :reader thrust
    :initarg :thrust
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetThrusterCurve-response (<GetThrusterCurve-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetThrusterCurve-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetThrusterCurve-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_thruster_manager-srv:<GetThrusterCurve-response> is deprecated: use uuv_thruster_manager-srv:GetThrusterCurve-response instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetThrusterCurve-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:input-val is deprecated.  Use uuv_thruster_manager-srv:input instead.")
  (input m))

(cl:ensure-generic-function 'thrust-val :lambda-list '(m))
(cl:defmethod thrust-val ((m <GetThrusterCurve-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_thruster_manager-srv:thrust-val is deprecated.  Use uuv_thruster_manager-srv:thrust instead.")
  (thrust m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetThrusterCurve-response>) ostream)
  "Serializes a message object of type '<GetThrusterCurve-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'input))))
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
   (cl:slot-value msg 'input))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'thrust))))
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
   (cl:slot-value msg 'thrust))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetThrusterCurve-response>) istream)
  "Deserializes a message object of type '<GetThrusterCurve-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'input) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'input)))
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
  (cl:setf (cl:slot-value msg 'thrust) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'thrust)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetThrusterCurve-response>)))
  "Returns string type for a service object of type '<GetThrusterCurve-response>"
  "uuv_thruster_manager/GetThrusterCurveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterCurve-response)))
  "Returns string type for a service object of type 'GetThrusterCurve-response"
  "uuv_thruster_manager/GetThrusterCurveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetThrusterCurve-response>)))
  "Returns md5sum for a message object of type '<GetThrusterCurve-response>"
  "93e0d8a78977b3619bdc09f290ef57fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetThrusterCurve-response)))
  "Returns md5sum for a message object of type 'GetThrusterCurve-response"
  "93e0d8a78977b3619bdc09f290ef57fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetThrusterCurve-response>)))
  "Returns full string definition for message of type '<GetThrusterCurve-response>"
  (cl:format cl:nil "float64[] input~%float64[] thrust~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetThrusterCurve-response)))
  "Returns full string definition for message of type 'GetThrusterCurve-response"
  (cl:format cl:nil "float64[] input~%float64[] thrust~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetThrusterCurve-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'input) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'thrust) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetThrusterCurve-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetThrusterCurve-response
    (cl:cons ':input (input msg))
    (cl:cons ':thrust (thrust msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetThrusterCurve)))
  'GetThrusterCurve-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetThrusterCurve)))
  'GetThrusterCurve-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetThrusterCurve)))
  "Returns string type for a service object of type '<GetThrusterCurve>"
  "uuv_thruster_manager/GetThrusterCurve")