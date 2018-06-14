;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_plugins_ros_msgs::Salinity)
  (if (not (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
    (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
  (shadow 'Salinity (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS")))
(unless (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::SALINITY")
  (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS::SALINITY"))

(in-package "ROS")
;;//! \htmlinclude Salinity.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*PSU*" (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::SALINITY"))
(shadow '*PSU* (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::SALINITY"))
(defconstant uuv_sensor_plugins_ros_msgs::Salinity::*PSU* "\"psu\"")
(intern "*PPM*" (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::SALINITY"))
(shadow '*PPM* (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::SALINITY"))
(defconstant uuv_sensor_plugins_ros_msgs::Salinity::*PPM* "\"ppm\"")
(intern "*PPT*" (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::SALINITY"))
(shadow '*PPT* (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::SALINITY"))
(defconstant uuv_sensor_plugins_ros_msgs::Salinity::*PPT* "\"ppt\"")
(defclass uuv_sensor_plugins_ros_msgs::Salinity
  :super ros::object
  :slots (_header _salinity _variance _unit ))

(defmethod uuv_sensor_plugins_ros_msgs::Salinity
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:salinity __salinity) 0.0)
    ((:variance __variance) 0.0)
    ((:unit __unit) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _salinity (float __salinity))
   (setq _variance (float __variance))
   (setq _unit (string __unit))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:salinity
   (&optional __salinity)
   (if __salinity (setq _salinity __salinity)) _salinity)
  (:variance
   (&optional __variance)
   (if __variance (setq _variance __variance)) _variance)
  (:unit
   (&optional __unit)
   (if __unit (setq _unit __unit)) _unit)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _salinity
    8
    ;; float64 _variance
    8
    ;; string _unit
    4 (length _unit)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _salinity
       (sys::poke _salinity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _variance
       (sys::poke _variance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _unit
       (write-long (length _unit) s) (princ _unit s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _salinity
     (setq _salinity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _variance
     (setq _variance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _unit
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _unit (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get uuv_sensor_plugins_ros_msgs::Salinity :md5sum-) "4d20de37b8b3a344b3f4c36f2192b257")
(setf (get uuv_sensor_plugins_ros_msgs::Salinity :datatype-) "uuv_sensor_plugins_ros_msgs/Salinity")
(setf (get uuv_sensor_plugins_ros_msgs::Salinity :definition-)
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

# Practical salinity units
string PSU=\"psu\"
# Parts per million
string PPM=\"ppm\"
# Parts per thousand
string PPT=\"ppt\"

std_msgs/Header header
float64 salinity
float64 variance
string unit
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



(provide :uuv_sensor_plugins_ros_msgs/Salinity "4d20de37b8b3a344b3f4c36f2192b257")


