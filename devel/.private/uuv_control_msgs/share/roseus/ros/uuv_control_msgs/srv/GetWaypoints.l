;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::GetWaypoints)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'GetWaypoints (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::GETWAYPOINTS")
  (make-package "UUV_CONTROL_MSGS::GETWAYPOINTS"))
(unless (find-package "UUV_CONTROL_MSGS::GETWAYPOINTSREQUEST")
  (make-package "UUV_CONTROL_MSGS::GETWAYPOINTSREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::GETWAYPOINTSRESPONSE")
  (make-package "UUV_CONTROL_MSGS::GETWAYPOINTSRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::GetWaypointsRequest
  :super ros::object
  :slots ())

(defmethod uuv_control_msgs::GetWaypointsRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass uuv_control_msgs::GetWaypointsResponse
  :super ros::object
  :slots (_waypoints ))

(defmethod uuv_control_msgs::GetWaypointsResponse
  (:init
   (&key
    ((:waypoints __waypoints) (let (r) (dotimes (i 0) (push (instance uuv_control_msgs::Waypoint :init) r)) r))
    )
   (send-super :init)
   (setq _waypoints __waypoints)
   self)
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
    ;; uuv_control_msgs/Waypoint[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uuv_control_msgs/Waypoint[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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

(defclass uuv_control_msgs::GetWaypoints
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::GetWaypoints :md5sum-) "7256248537b12c726720758df9a72413")
(setf (get uuv_control_msgs::GetWaypoints :datatype-) "uuv_control_msgs/GetWaypoints")
(setf (get uuv_control_msgs::GetWaypoints :request) uuv_control_msgs::GetWaypointsRequest)
(setf (get uuv_control_msgs::GetWaypoints :response) uuv_control_msgs::GetWaypointsResponse)

(defmethod uuv_control_msgs::GetWaypointsRequest
  (:response () (instance uuv_control_msgs::GetWaypointsResponse :init)))

(setf (get uuv_control_msgs::GetWaypointsRequest :md5sum-) "7256248537b12c726720758df9a72413")
(setf (get uuv_control_msgs::GetWaypointsRequest :datatype-) "uuv_control_msgs/GetWaypointsRequest")
(setf (get uuv_control_msgs::GetWaypointsRequest :definition-)
      "














---
uuv_control_msgs/Waypoint[] waypoints


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
")

(setf (get uuv_control_msgs::GetWaypointsResponse :md5sum-) "7256248537b12c726720758df9a72413")
(setf (get uuv_control_msgs::GetWaypointsResponse :datatype-) "uuv_control_msgs/GetWaypointsResponse")
(setf (get uuv_control_msgs::GetWaypointsResponse :definition-)
      "














---
uuv_control_msgs/Waypoint[] waypoints


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
")



(provide :uuv_control_msgs/GetWaypoints "7256248537b12c726720758df9a72413")


