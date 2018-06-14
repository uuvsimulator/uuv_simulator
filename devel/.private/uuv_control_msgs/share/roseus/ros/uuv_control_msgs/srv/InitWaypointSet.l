;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::InitWaypointSet)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'InitWaypointSet (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSET")
  (make-package "UUV_CONTROL_MSGS::INITWAYPOINTSET"))
(unless (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSETREQUEST")
  (make-package "UUV_CONTROL_MSGS::INITWAYPOINTSETREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSETRESPONSE")
  (make-package "UUV_CONTROL_MSGS::INITWAYPOINTSETRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass uuv_control_msgs::InitWaypointSetRequest
  :super ros::object
  :slots (_start_time _start_now _waypoints _max_forward_speed _heading_offset ))

(defmethod uuv_control_msgs::InitWaypointSetRequest
  (:init
   (&key
    ((:start_time __start_time) (instance std_msgs::Time :init))
    ((:start_now __start_now) nil)
    ((:waypoints __waypoints) (let (r) (dotimes (i 0) (push (instance uuv_control_msgs::Waypoint :init) r)) r))
    ((:max_forward_speed __max_forward_speed) 0.0)
    ((:heading_offset __heading_offset) 0.0)
    )
   (send-super :init)
   (setq _start_time __start_time)
   (setq _start_now __start_now)
   (setq _waypoints __waypoints)
   (setq _max_forward_speed (float __max_forward_speed))
   (setq _heading_offset (float __heading_offset))
   self)
  (:start_time
   (&rest __start_time)
   (if (keywordp (car __start_time))
       (send* _start_time __start_time)
     (progn
       (if __start_time (setq _start_time (car __start_time)))
       _start_time)))
  (:start_now
   (&optional __start_now)
   (if __start_now (setq _start_now __start_now)) _start_now)
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:max_forward_speed
   (&optional __max_forward_speed)
   (if __max_forward_speed (setq _max_forward_speed __max_forward_speed)) _max_forward_speed)
  (:heading_offset
   (&optional __heading_offset)
   (if __heading_offset (setq _heading_offset __heading_offset)) _heading_offset)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Time _start_time
    (send _start_time :serialization-length)
    ;; bool _start_now
    1
    ;; uuv_control_msgs/Waypoint[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ;; float64 _max_forward_speed
    8
    ;; float64 _heading_offset
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Time _start_time
       (send _start_time :serialize s)
     ;; bool _start_now
       (if _start_now (write-byte -1 s) (write-byte 0 s))
     ;; uuv_control_msgs/Waypoint[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;; float64 _max_forward_speed
       (sys::poke _max_forward_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_offset
       (sys::poke _heading_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Time _start_time
     (send _start_time :deserialize buf ptr-) (incf ptr- (send _start_time :serialization-length))
   ;; bool _start_now
     (setq _start_now (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uuv_control_msgs/Waypoint[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance uuv_control_msgs::Waypoint :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64 _max_forward_speed
     (setq _max_forward_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_offset
     (setq _heading_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_control_msgs::InitWaypointSetResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::InitWaypointSetResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uuv_control_msgs::InitWaypointSet
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::InitWaypointSet :md5sum-) "a5fe64b7879e685af95ae8210e45b1c6")
(setf (get uuv_control_msgs::InitWaypointSet :datatype-) "uuv_control_msgs/InitWaypointSet")
(setf (get uuv_control_msgs::InitWaypointSet :request) uuv_control_msgs::InitWaypointSetRequest)
(setf (get uuv_control_msgs::InitWaypointSet :response) uuv_control_msgs::InitWaypointSetResponse)

(defmethod uuv_control_msgs::InitWaypointSetRequest
  (:response () (instance uuv_control_msgs::InitWaypointSetResponse :init)))

(setf (get uuv_control_msgs::InitWaypointSetRequest :md5sum-) "a5fe64b7879e685af95ae8210e45b1c6")
(setf (get uuv_control_msgs::InitWaypointSetRequest :datatype-) "uuv_control_msgs/InitWaypointSetRequest")
(setf (get uuv_control_msgs::InitWaypointSetRequest :definition-)
      "














std_msgs/Time start_time
bool start_now
uuv_control_msgs/Waypoint[] waypoints
float64 max_forward_speed
float64 heading_offset

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
---
bool success

")

(setf (get uuv_control_msgs::InitWaypointSetResponse :md5sum-) "a5fe64b7879e685af95ae8210e45b1c6")
(setf (get uuv_control_msgs::InitWaypointSetResponse :datatype-) "uuv_control_msgs/InitWaypointSetResponse")
(setf (get uuv_control_msgs::InitWaypointSetResponse :definition-)
      "














std_msgs/Time start_time
bool start_now
uuv_control_msgs/Waypoint[] waypoints
float64 max_forward_speed
float64 heading_offset

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
---
bool success

")



(provide :uuv_control_msgs/InitWaypointSet "a5fe64b7879e685af95ae8210e45b1c6")


