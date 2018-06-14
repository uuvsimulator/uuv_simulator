;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_plugins_ros_msgs::DVL)
  (if (not (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
    (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
  (shadow 'DVL (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS")))
(unless (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::DVL")
  (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS::DVL"))

(in-package "ROS")
;;//! \htmlinclude DVL.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uuv_sensor_plugins_ros_msgs::DVL
  :super ros::object
  :slots (_header _velocity _velocity_covariance _altitude _beams ))

(defmethod uuv_sensor_plugins_ros_msgs::DVL
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:velocity_covariance __velocity_covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:altitude __altitude) 0.0)
    ((:beams __beams) (let (r) (dotimes (i 0) (push (instance uuv_sensor_plugins_ros_msgs::DVLBeam :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _velocity __velocity)
   (setq _velocity_covariance __velocity_covariance)
   (setq _altitude (float __altitude))
   (setq _beams __beams)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:velocity_covariance
   (&optional __velocity_covariance)
   (if __velocity_covariance (setq _velocity_covariance __velocity_covariance)) _velocity_covariance)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:beams
   (&rest __beams)
   (if (keywordp (car __beams))
       (send* _beams __beams)
     (progn
       (if __beams (setq _beams (car __beams)))
       _beams)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; float64[9] _velocity_covariance
    (* 8    9)
    ;; float64 _altitude
    8
    ;; uuv_sensor_plugins_ros_msgs/DVLBeam[] _beams
    (apply #'+ (send-all _beams :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; float64[9] _velocity_covariance
     (dotimes (i 9)
       (sys::poke (elt _velocity_covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uuv_sensor_plugins_ros_msgs/DVLBeam[] _beams
     (write-long (length _beams) s)
     (dolist (elem _beams)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; float64[9] _velocity_covariance
   (dotimes (i (length _velocity_covariance))
     (setf (elt _velocity_covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uuv_sensor_plugins_ros_msgs/DVLBeam[] _beams
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _beams (let (r) (dotimes (i n) (push (instance uuv_sensor_plugins_ros_msgs::DVLBeam :init) r)) r))
     (dolist (elem- _beams)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get uuv_sensor_plugins_ros_msgs::DVL :md5sum-) "02bba6182b6f271447d7f88473256572")
(setf (get uuv_sensor_plugins_ros_msgs::DVL :datatype-) "uuv_sensor_plugins_ros_msgs/DVL")
(setf (get uuv_sensor_plugins_ros_msgs::DVL :definition-)
      "# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This is a message to hold data from a DVL sensor (Doppler Velocity Log).
#
# Distances are in [m], velocities in [m/s]
#
# If the covariance is known, it should be filled.
# If it is unknown, it should be set to all zeros.
# If a measurement was invalid, its covariance should be set to -1 so it can be
# disregarded.
#
# DVLBeams are optional. If they are set they contain individual ranges and 1D
# doppler velocity estimates orthogonal to the ray.

Header header
# Measured velocity [m/s]
geometry_msgs/Vector3 velocity
# Row major, xyz axes
float64[9] velocity_covariance
# Altitude of the vehicle
float64 altitude
DVLBeam[] beams

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

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: uuv_sensor_plugins_ros_msgs/DVLBeam
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# measured range [m] or < 0 if invalid
float64 range
float64 range_covariance

float64 velocity # measured velocity [m/s] of corr. beam
float64 velocity_covariance

# Beam link pose wrt DVL link frame
geometry_msgs/PoseStamped pose

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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

")



(provide :uuv_sensor_plugins_ros_msgs/DVL "02bba6182b6f271447d7f88473256572")


