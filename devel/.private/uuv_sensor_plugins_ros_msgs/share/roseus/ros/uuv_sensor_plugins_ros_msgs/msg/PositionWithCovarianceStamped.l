;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_plugins_ros_msgs::PositionWithCovarianceStamped)
  (if (not (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
    (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
  (shadow 'PositionWithCovarianceStamped (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS")))
(unless (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::POSITIONWITHCOVARIANCESTAMPED")
  (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS::POSITIONWITHCOVARIANCESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude PositionWithCovarianceStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uuv_sensor_plugins_ros_msgs::PositionWithCovarianceStamped
  :super ros::object
  :slots (_header _pos ))

(defmethod uuv_sensor_plugins_ros_msgs::PositionWithCovarianceStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pos __pos) (instance uuv_sensor_plugins_ros_msgs::PositionWithCovariance :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _pos __pos)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uuv_sensor_plugins_ros_msgs/PositionWithCovariance _pos
    (send _pos :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uuv_sensor_plugins_ros_msgs/PositionWithCovariance _pos
       (send _pos :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uuv_sensor_plugins_ros_msgs/PositionWithCovariance _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;;
   self)
  )

(setf (get uuv_sensor_plugins_ros_msgs::PositionWithCovarianceStamped :md5sum-) "ef0ae60585e532ef356441a1a701f864")
(setf (get uuv_sensor_plugins_ros_msgs::PositionWithCovarianceStamped :datatype-) "uuv_sensor_plugins_ros_msgs/PositionWithCovarianceStamped")
(setf (get uuv_sensor_plugins_ros_msgs::PositionWithCovarianceStamped :definition-)
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

# This expresses an estimated position with a reference coordinate frame and
# timestamp

Header header
PositionWithCovariance pos

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
MSG: uuv_sensor_plugins_ros_msgs/PositionWithCovariance
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

# This represents a position in free space with uncertainty.

geometry_msgs/Point pos

# Row-major representation of the 3x3 covariance matrix
float64[9] covariance

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :uuv_sensor_plugins_ros_msgs/PositionWithCovarianceStamped "ef0ae60585e532ef356441a1a701f864")


