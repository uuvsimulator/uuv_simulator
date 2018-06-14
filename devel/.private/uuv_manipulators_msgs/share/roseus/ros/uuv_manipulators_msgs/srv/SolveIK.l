;; Auto-generated. Do not edit!


(when (boundp 'uuv_manipulators_msgs::SolveIK)
  (if (not (find-package "UUV_MANIPULATORS_MSGS"))
    (make-package "UUV_MANIPULATORS_MSGS"))
  (shadow 'SolveIK (find-package "UUV_MANIPULATORS_MSGS")))
(unless (find-package "UUV_MANIPULATORS_MSGS::SOLVEIK")
  (make-package "UUV_MANIPULATORS_MSGS::SOLVEIK"))
(unless (find-package "UUV_MANIPULATORS_MSGS::SOLVEIKREQUEST")
  (make-package "UUV_MANIPULATORS_MSGS::SOLVEIKREQUEST"))
(unless (find-package "UUV_MANIPULATORS_MSGS::SOLVEIKRESPONSE")
  (make-package "UUV_MANIPULATORS_MSGS::SOLVEIKRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass uuv_manipulators_msgs::SolveIKRequest
  :super ros::object
  :slots (_pose _seed_angles ))

(defmethod uuv_manipulators_msgs::SolveIKRequest
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:seed_angles __seed_angles) (instance sensor_msgs::JointState :init))
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _seed_angles __seed_angles)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:seed_angles
   (&rest __seed_angles)
   (if (keywordp (car __seed_angles))
       (send* _seed_angles __seed_angles)
     (progn
       (if __seed_angles (setq _seed_angles (car __seed_angles)))
       _seed_angles)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; sensor_msgs/JointState _seed_angles
    (send _seed_angles :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; sensor_msgs/JointState _seed_angles
       (send _seed_angles :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; sensor_msgs/JointState _seed_angles
     (send _seed_angles :deserialize buf ptr-) (incf ptr- (send _seed_angles :serialization-length))
   ;;
   self)
  )

(defclass uuv_manipulators_msgs::SolveIKResponse
  :super ros::object
  :slots (_joints _isValid ))

(defmethod uuv_manipulators_msgs::SolveIKResponse
  (:init
   (&key
    ((:joints __joints) (instance sensor_msgs::JointState :init))
    ((:isValid __isValid) nil)
    )
   (send-super :init)
   (setq _joints __joints)
   (setq _isValid __isValid)
   self)
  (:joints
   (&rest __joints)
   (if (keywordp (car __joints))
       (send* _joints __joints)
     (progn
       (if __joints (setq _joints (car __joints)))
       _joints)))
  (:isValid
   (&optional __isValid)
   (if __isValid (setq _isValid __isValid)) _isValid)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/JointState _joints
    (send _joints :serialization-length)
    ;; bool _isValid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/JointState _joints
       (send _joints :serialize s)
     ;; bool _isValid
       (if _isValid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/JointState _joints
     (send _joints :deserialize buf ptr-) (incf ptr- (send _joints :serialization-length))
   ;; bool _isValid
     (setq _isValid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uuv_manipulators_msgs::SolveIK
  :super ros::object
  :slots ())

(setf (get uuv_manipulators_msgs::SolveIK :md5sum-) "135cec71206c75b4d25a0c184fe860d8")
(setf (get uuv_manipulators_msgs::SolveIK :datatype-) "uuv_manipulators_msgs/SolveIK")
(setf (get uuv_manipulators_msgs::SolveIK :request) uuv_manipulators_msgs::SolveIKRequest)
(setf (get uuv_manipulators_msgs::SolveIK :response) uuv_manipulators_msgs::SolveIKResponse)

(defmethod uuv_manipulators_msgs::SolveIKRequest
  (:response () (instance uuv_manipulators_msgs::SolveIKResponse :init)))

(setf (get uuv_manipulators_msgs::SolveIKRequest :md5sum-) "135cec71206c75b4d25a0c184fe860d8")
(setf (get uuv_manipulators_msgs::SolveIKRequest :datatype-) "uuv_manipulators_msgs/SolveIKRequest")
(setf (get uuv_manipulators_msgs::SolveIKRequest :definition-)
      "geometry_msgs/Pose pose
sensor_msgs/JointState seed_angles

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
---
sensor_msgs/JointState joints
bool isValid


================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
")

(setf (get uuv_manipulators_msgs::SolveIKResponse :md5sum-) "135cec71206c75b4d25a0c184fe860d8")
(setf (get uuv_manipulators_msgs::SolveIKResponse :datatype-) "uuv_manipulators_msgs/SolveIKResponse")
(setf (get uuv_manipulators_msgs::SolveIKResponse :definition-)
      "geometry_msgs/Pose pose
sensor_msgs/JointState seed_angles

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
---
sensor_msgs/JointState joints
bool isValid


================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
")



(provide :uuv_manipulators_msgs/SolveIK "135cec71206c75b4d25a0c184fe860d8")


