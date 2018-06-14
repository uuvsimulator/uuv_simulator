; Auto-generated. Do not edit!


(cl:in-package uuv_manipulators_msgs-srv)


;//! \htmlinclude SolveIK-request.msg.html

(cl:defclass <SolveIK-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (seed_angles
    :reader seed_angles
    :initarg :seed_angles
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass SolveIK-request (<SolveIK-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SolveIK-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SolveIK-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_manipulators_msgs-srv:<SolveIK-request> is deprecated: use uuv_manipulators_msgs-srv:SolveIK-request instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <SolveIK-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-srv:pose-val is deprecated.  Use uuv_manipulators_msgs-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'seed_angles-val :lambda-list '(m))
(cl:defmethod seed_angles-val ((m <SolveIK-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-srv:seed_angles-val is deprecated.  Use uuv_manipulators_msgs-srv:seed_angles instead.")
  (seed_angles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SolveIK-request>) ostream)
  "Serializes a message object of type '<SolveIK-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'seed_angles) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SolveIK-request>) istream)
  "Deserializes a message object of type '<SolveIK-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'seed_angles) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SolveIK-request>)))
  "Returns string type for a service object of type '<SolveIK-request>"
  "uuv_manipulators_msgs/SolveIKRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolveIK-request)))
  "Returns string type for a service object of type 'SolveIK-request"
  "uuv_manipulators_msgs/SolveIKRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SolveIK-request>)))
  "Returns md5sum for a message object of type '<SolveIK-request>"
  "135cec71206c75b4d25a0c184fe860d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SolveIK-request)))
  "Returns md5sum for a message object of type 'SolveIK-request"
  "135cec71206c75b4d25a0c184fe860d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SolveIK-request>)))
  "Returns full string definition for message of type '<SolveIK-request>"
  (cl:format cl:nil "geometry_msgs/Pose pose~%sensor_msgs/JointState seed_angles~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SolveIK-request)))
  "Returns full string definition for message of type 'SolveIK-request"
  (cl:format cl:nil "geometry_msgs/Pose pose~%sensor_msgs/JointState seed_angles~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SolveIK-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'seed_angles))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SolveIK-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SolveIK-request
    (cl:cons ':pose (pose msg))
    (cl:cons ':seed_angles (seed_angles msg))
))
;//! \htmlinclude SolveIK-response.msg.html

(cl:defclass <SolveIK-response> (roslisp-msg-protocol:ros-message)
  ((joints
    :reader joints
    :initarg :joints
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState))
   (isValid
    :reader isValid
    :initarg :isValid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SolveIK-response (<SolveIK-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SolveIK-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SolveIK-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_manipulators_msgs-srv:<SolveIK-response> is deprecated: use uuv_manipulators_msgs-srv:SolveIK-response instead.")))

(cl:ensure-generic-function 'joints-val :lambda-list '(m))
(cl:defmethod joints-val ((m <SolveIK-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-srv:joints-val is deprecated.  Use uuv_manipulators_msgs-srv:joints instead.")
  (joints m))

(cl:ensure-generic-function 'isValid-val :lambda-list '(m))
(cl:defmethod isValid-val ((m <SolveIK-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_manipulators_msgs-srv:isValid-val is deprecated.  Use uuv_manipulators_msgs-srv:isValid instead.")
  (isValid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SolveIK-response>) ostream)
  "Serializes a message object of type '<SolveIK-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joints) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isValid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SolveIK-response>) istream)
  "Deserializes a message object of type '<SolveIK-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joints) istream)
    (cl:setf (cl:slot-value msg 'isValid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SolveIK-response>)))
  "Returns string type for a service object of type '<SolveIK-response>"
  "uuv_manipulators_msgs/SolveIKResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolveIK-response)))
  "Returns string type for a service object of type 'SolveIK-response"
  "uuv_manipulators_msgs/SolveIKResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SolveIK-response>)))
  "Returns md5sum for a message object of type '<SolveIK-response>"
  "135cec71206c75b4d25a0c184fe860d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SolveIK-response)))
  "Returns md5sum for a message object of type 'SolveIK-response"
  "135cec71206c75b4d25a0c184fe860d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SolveIK-response>)))
  "Returns full string definition for message of type '<SolveIK-response>"
  (cl:format cl:nil "sensor_msgs/JointState joints~%bool isValid~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SolveIK-response)))
  "Returns full string definition for message of type 'SolveIK-response"
  (cl:format cl:nil "sensor_msgs/JointState joints~%bool isValid~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SolveIK-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joints))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SolveIK-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SolveIK-response
    (cl:cons ':joints (joints msg))
    (cl:cons ':isValid (isValid msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SolveIK)))
  'SolveIK-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SolveIK)))
  'SolveIK-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SolveIK)))
  "Returns string type for a service object of type '<SolveIK>"
  "uuv_manipulators_msgs/SolveIK")