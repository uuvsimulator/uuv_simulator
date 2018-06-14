;; Auto-generated. Do not edit!


(when (boundp 'uuv_gazebo_ros_plugins_msgs::FloatStamped)
  (if (not (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
    (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
  (shadow 'FloatStamped (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::FLOATSTAMPED")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::FLOATSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude FloatStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uuv_gazebo_ros_plugins_msgs::FloatStamped
  :super ros::object
  :slots (_header _data ))

(defmethod uuv_gazebo_ros_plugins_msgs::FloatStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _data (float __data))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _data
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _data
       (sys::poke _data (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _data
     (setq _data (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get uuv_gazebo_ros_plugins_msgs::FloatStamped :md5sum-) "e6c99c37e6f9fe98e071d524cc164e65")
(setf (get uuv_gazebo_ros_plugins_msgs::FloatStamped :datatype-) "uuv_gazebo_ros_plugins_msgs/FloatStamped")
(setf (get uuv_gazebo_ros_plugins_msgs::FloatStamped :definition-)
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

Header header
float64 data

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



(provide :uuv_gazebo_ros_plugins_msgs/FloatStamped "e6c99c37e6f9fe98e071d524cc164e65")


