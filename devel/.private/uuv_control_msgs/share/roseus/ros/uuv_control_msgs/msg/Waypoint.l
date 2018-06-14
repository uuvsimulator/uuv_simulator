;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::Waypoint)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'Waypoint (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::WAYPOINT")
  (make-package "UUV_CONTROL_MSGS::WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Waypoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uuv_control_msgs::Waypoint
  :super ros::object
  :slots (_header _point _max_forward_speed _heading_offset _use_fixed_heading _radius_of_acceptance ))

(defmethod uuv_control_msgs::Waypoint
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:max_forward_speed __max_forward_speed) 0.0)
    ((:heading_offset __heading_offset) 0.0)
    ((:use_fixed_heading __use_fixed_heading) nil)
    ((:radius_of_acceptance __radius_of_acceptance) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _point __point)
   (setq _max_forward_speed (float __max_forward_speed))
   (setq _heading_offset (float __heading_offset))
   (setq _use_fixed_heading __use_fixed_heading)
   (setq _radius_of_acceptance (float __radius_of_acceptance))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:max_forward_speed
   (&optional __max_forward_speed)
   (if __max_forward_speed (setq _max_forward_speed __max_forward_speed)) _max_forward_speed)
  (:heading_offset
   (&optional __heading_offset)
   (if __heading_offset (setq _heading_offset __heading_offset)) _heading_offset)
  (:use_fixed_heading
   (&optional __use_fixed_heading)
   (if __use_fixed_heading (setq _use_fixed_heading __use_fixed_heading)) _use_fixed_heading)
  (:radius_of_acceptance
   (&optional __radius_of_acceptance)
   (if __radius_of_acceptance (setq _radius_of_acceptance __radius_of_acceptance)) _radius_of_acceptance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; float64 _max_forward_speed
    8
    ;; float64 _heading_offset
    8
    ;; bool _use_fixed_heading
    1
    ;; float64 _radius_of_acceptance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; float64 _max_forward_speed
       (sys::poke _max_forward_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_offset
       (sys::poke _heading_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _use_fixed_heading
       (if _use_fixed_heading (write-byte -1 s) (write-byte 0 s))
     ;; float64 _radius_of_acceptance
       (sys::poke _radius_of_acceptance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; float64 _max_forward_speed
     (setq _max_forward_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_offset
     (setq _heading_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _use_fixed_heading
     (setq _use_fixed_heading (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _radius_of_acceptance
     (setq _radius_of_acceptance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get uuv_control_msgs::Waypoint :md5sum-) "0efb7fda1b5980152de94b6064a5cf35")
(setf (get uuv_control_msgs::Waypoint :datatype-) "uuv_control_msgs/Waypoint")
(setf (get uuv_control_msgs::Waypoint :definition-)
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
geometry_msgs/Point point
float64 max_forward_speed
float64 heading_offset
bool use_fixed_heading
float64 radius_of_acceptance

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :uuv_control_msgs/Waypoint "0efb7fda1b5980152de94b6064a5cf35")


