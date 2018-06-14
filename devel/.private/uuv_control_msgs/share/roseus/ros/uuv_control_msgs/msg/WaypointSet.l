;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::WaypointSet)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'WaypointSet (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::WAYPOINTSET")
  (make-package "UUV_CONTROL_MSGS::WAYPOINTSET"))

(in-package "ROS")
;;//! \htmlinclude WaypointSet.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uuv_control_msgs::WaypointSet
  :super ros::object
  :slots (_header _start_time _waypoints ))

(defmethod uuv_control_msgs::WaypointSet
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:start_time __start_time) (instance std_msgs::Time :init))
    ((:waypoints __waypoints) (let (r) (dotimes (i 0) (push (instance uuv_control_msgs::Waypoint :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _start_time __start_time)
   (setq _waypoints __waypoints)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:start_time
   (&rest __start_time)
   (if (keywordp (car __start_time))
       (send* _start_time __start_time)
     (progn
       (if __start_time (setq _start_time (car __start_time)))
       _start_time)))
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Time _start_time
    (send _start_time :serialization-length)
    ;; uuv_control_msgs/Waypoint[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Time _start_time
       (send _start_time :serialize s)
     ;; uuv_control_msgs/Waypoint[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Time _start_time
     (send _start_time :deserialize buf ptr-) (incf ptr- (send _start_time :serialization-length))
   ;; uuv_control_msgs/Waypoint[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance uuv_control_msgs::Waypoint :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get uuv_control_msgs::WaypointSet :md5sum-) "a946aaa8d17e35fe5a52e4f7ac482b5d")
(setf (get uuv_control_msgs::WaypointSet :datatype-) "uuv_control_msgs/WaypointSet")
(setf (get uuv_control_msgs::WaypointSet :definition-)
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
std_msgs/Time start_time
uuv_control_msgs/Waypoint[] waypoints

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
MSG: std_msgs/Time
time data

================================================================================
MSG: uuv_control_msgs/Waypoint
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

std_msgs/Header header
geometry_msgs/Point point
float64 max_forward_speed
float64 heading_offset
bool use_fixed_heading
float64 radius_of_acceptance

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :uuv_control_msgs/WaypointSet "a946aaa8d17e35fe5a52e4f7ac482b5d")


