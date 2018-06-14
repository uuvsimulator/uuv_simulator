;; Auto-generated. Do not edit!


(when (boundp 'uuv_auv_control_allocator::AUVCommand)
  (if (not (find-package "UUV_AUV_CONTROL_ALLOCATOR"))
    (make-package "UUV_AUV_CONTROL_ALLOCATOR"))
  (shadow 'AUVCommand (find-package "UUV_AUV_CONTROL_ALLOCATOR")))
(unless (find-package "UUV_AUV_CONTROL_ALLOCATOR::AUVCOMMAND")
  (make-package "UUV_AUV_CONTROL_ALLOCATOR::AUVCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude AUVCommand.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uuv_auv_control_allocator::AUVCommand
  :super ros::object
  :slots (_header _surge_speed _command ))

(defmethod uuv_auv_control_allocator::AUVCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:surge_speed __surge_speed) 0.0)
    ((:command __command) (instance geometry_msgs::Wrench :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _surge_speed (float __surge_speed))
   (setq _command __command)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:surge_speed
   (&optional __surge_speed)
   (if __surge_speed (setq _surge_speed __surge_speed)) _surge_speed)
  (:command
   (&rest __command)
   (if (keywordp (car __command))
       (send* _command __command)
     (progn
       (if __command (setq _command (car __command)))
       _command)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _surge_speed
    8
    ;; geometry_msgs/Wrench _command
    (send _command :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _surge_speed
       (sys::poke _surge_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Wrench _command
       (send _command :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _surge_speed
     (setq _surge_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Wrench _command
     (send _command :deserialize buf ptr-) (incf ptr- (send _command :serialization-length))
   ;;
   self)
  )

(setf (get uuv_auv_control_allocator::AUVCommand :md5sum-) "9d7c962f08b7f118399273df23351e7c")
(setf (get uuv_auv_control_allocator::AUVCommand :datatype-) "uuv_auv_control_allocator/AUVCommand")
(setf (get uuv_auv_control_allocator::AUVCommand :definition-)
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

std_msgs/Header header
float64 surge_speed
geometry_msgs/Wrench command
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
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

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
")



(provide :uuv_auv_control_allocator/AUVCommand "9d7c962f08b7f118399273df23351e7c")


