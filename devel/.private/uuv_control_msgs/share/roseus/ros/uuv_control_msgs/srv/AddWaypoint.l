;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::AddWaypoint)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'AddWaypoint (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::ADDWAYPOINT")
  (make-package "UUV_CONTROL_MSGS::ADDWAYPOINT"))
(unless (find-package "UUV_CONTROL_MSGS::ADDWAYPOINTREQUEST")
  (make-package "UUV_CONTROL_MSGS::ADDWAYPOINTREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::ADDWAYPOINTRESPONSE")
  (make-package "UUV_CONTROL_MSGS::ADDWAYPOINTRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::AddWaypointRequest
  :super ros::object
  :slots (_waypoint ))

(defmethod uuv_control_msgs::AddWaypointRequest
  (:init
   (&key
    ((:waypoint __waypoint) (instance uuv_control_msgs::Waypoint :init))
    )
   (send-super :init)
   (setq _waypoint __waypoint)
   self)
  (:waypoint
   (&rest __waypoint)
   (if (keywordp (car __waypoint))
       (send* _waypoint __waypoint)
     (progn
       (if __waypoint (setq _waypoint (car __waypoint)))
       _waypoint)))
  (:serialization-length
   ()
   (+
    ;; uuv_control_msgs/Waypoint _waypoint
    (send _waypoint :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uuv_control_msgs/Waypoint _waypoint
       (send _waypoint :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uuv_control_msgs/Waypoint _waypoint
     (send _waypoint :deserialize buf ptr-) (incf ptr- (send _waypoint :serialization-length))
   ;;
   self)
  )

(defclass uuv_control_msgs::AddWaypointResponse
  :super ros::object
  :slots (_success _waypoints ))

(defmethod uuv_control_msgs::AddWaypointResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:waypoints __waypoints) (let (r) (dotimes (i 0) (push (instance uuv_control_msgs::Waypoint :init) r)) r))
    )
   (send-super :init)
   (setq _success __success)
   (setq _waypoints __waypoints)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
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
    ;; bool _success
    1
    ;; uuv_control_msgs/Waypoint[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; uuv_control_msgs/Waypoint[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
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

(defclass uuv_control_msgs::AddWaypoint
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::AddWaypoint :md5sum-) "e853788769392728a6445812f447d75e")
(setf (get uuv_control_msgs::AddWaypoint :datatype-) "uuv_control_msgs/AddWaypoint")
(setf (get uuv_control_msgs::AddWaypoint :request) uuv_control_msgs::AddWaypointRequest)
(setf (get uuv_control_msgs::AddWaypoint :response) uuv_control_msgs::AddWaypointResponse)

(defmethod uuv_control_msgs::AddWaypointRequest
  (:response () (instance uuv_control_msgs::AddWaypointResponse :init)))

(setf (get uuv_control_msgs::AddWaypointRequest :md5sum-) "e853788769392728a6445812f447d75e")
(setf (get uuv_control_msgs::AddWaypointRequest :datatype-) "uuv_control_msgs/AddWaypointRequest")
(setf (get uuv_control_msgs::AddWaypointRequest :definition-)
      "














uuv_control_msgs/Waypoint waypoint

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

(setf (get uuv_control_msgs::AddWaypointResponse :md5sum-) "e853788769392728a6445812f447d75e")
(setf (get uuv_control_msgs::AddWaypointResponse :datatype-) "uuv_control_msgs/AddWaypointResponse")
(setf (get uuv_control_msgs::AddWaypointResponse :definition-)
      "














uuv_control_msgs/Waypoint waypoint

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



(provide :uuv_control_msgs/AddWaypoint "e853788769392728a6445812f447d75e")


