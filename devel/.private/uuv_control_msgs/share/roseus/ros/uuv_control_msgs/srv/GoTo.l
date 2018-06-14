;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::GoTo)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'GoTo (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::GOTO")
  (make-package "UUV_CONTROL_MSGS::GOTO"))
(unless (find-package "UUV_CONTROL_MSGS::GOTOREQUEST")
  (make-package "UUV_CONTROL_MSGS::GOTOREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::GOTORESPONSE")
  (make-package "UUV_CONTROL_MSGS::GOTORESPONSE"))

(in-package "ROS")





(intern "*LIPB*" (find-package "UUV_CONTROL_MSGS::GOTOREQUEST"))
(shadow '*LIPB* (find-package "UUV_CONTROL_MSGS::GOTOREQUEST"))
(defconstant uuv_control_msgs::GoToRequest::*LIPB* "'lipb'")
(intern "*CUBIC*" (find-package "UUV_CONTROL_MSGS::GOTOREQUEST"))
(shadow '*CUBIC* (find-package "UUV_CONTROL_MSGS::GOTOREQUEST"))
(defconstant uuv_control_msgs::GoToRequest::*CUBIC* "'cubic'")
(intern "*DUBINS*" (find-package "UUV_CONTROL_MSGS::GOTOREQUEST"))
(shadow '*DUBINS* (find-package "UUV_CONTROL_MSGS::GOTOREQUEST"))
(defconstant uuv_control_msgs::GoToRequest::*DUBINS* "'dubins'")
(intern "*LINEAR*" (find-package "UUV_CONTROL_MSGS::GOTOREQUEST"))
(shadow '*LINEAR* (find-package "UUV_CONTROL_MSGS::GOTOREQUEST"))
(defconstant uuv_control_msgs::GoToRequest::*LINEAR* "'linear'")
(defclass uuv_control_msgs::GoToRequest
  :super ros::object
  :slots (_waypoint _max_forward_speed _interpolator ))

(defmethod uuv_control_msgs::GoToRequest
  (:init
   (&key
    ((:waypoint __waypoint) (instance uuv_control_msgs::Waypoint :init))
    ((:max_forward_speed __max_forward_speed) 0.0)
    ((:interpolator __interpolator) "")
    )
   (send-super :init)
   (setq _waypoint __waypoint)
   (setq _max_forward_speed (float __max_forward_speed))
   (setq _interpolator (string __interpolator))
   self)
  (:waypoint
   (&rest __waypoint)
   (if (keywordp (car __waypoint))
       (send* _waypoint __waypoint)
     (progn
       (if __waypoint (setq _waypoint (car __waypoint)))
       _waypoint)))
  (:max_forward_speed
   (&optional __max_forward_speed)
   (if __max_forward_speed (setq _max_forward_speed __max_forward_speed)) _max_forward_speed)
  (:interpolator
   (&optional __interpolator)
   (if __interpolator (setq _interpolator __interpolator)) _interpolator)
  (:serialization-length
   ()
   (+
    ;; uuv_control_msgs/Waypoint _waypoint
    (send _waypoint :serialization-length)
    ;; float64 _max_forward_speed
    8
    ;; string _interpolator
    4 (length _interpolator)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uuv_control_msgs/Waypoint _waypoint
       (send _waypoint :serialize s)
     ;; float64 _max_forward_speed
       (sys::poke _max_forward_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _interpolator
       (write-long (length _interpolator) s) (princ _interpolator s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uuv_control_msgs/Waypoint _waypoint
     (send _waypoint :deserialize buf ptr-) (incf ptr- (send _waypoint :serialization-length))
   ;; float64 _max_forward_speed
     (setq _max_forward_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _interpolator
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _interpolator (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uuv_control_msgs::GoToResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::GoToResponse
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

(defclass uuv_control_msgs::GoTo
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::GoTo :md5sum-) "408446fa9ec1d90b38a7053e3dd0ad47")
(setf (get uuv_control_msgs::GoTo :datatype-) "uuv_control_msgs/GoTo")
(setf (get uuv_control_msgs::GoTo :request) uuv_control_msgs::GoToRequest)
(setf (get uuv_control_msgs::GoTo :response) uuv_control_msgs::GoToResponse)

(defmethod uuv_control_msgs::GoToRequest
  (:response () (instance uuv_control_msgs::GoToResponse :init)))

(setf (get uuv_control_msgs::GoToRequest :md5sum-) "408446fa9ec1d90b38a7053e3dd0ad47")
(setf (get uuv_control_msgs::GoToRequest :datatype-) "uuv_control_msgs/GoToRequest")
(setf (get uuv_control_msgs::GoToRequest :definition-)
      "














string LIPB='lipb'
string CUBIC='cubic'
string DUBINS='dubins'
string LINEAR='linear'

uuv_control_msgs/Waypoint waypoint
float64 max_forward_speed
string interpolator

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

(setf (get uuv_control_msgs::GoToResponse :md5sum-) "408446fa9ec1d90b38a7053e3dd0ad47")
(setf (get uuv_control_msgs::GoToResponse :datatype-) "uuv_control_msgs/GoToResponse")
(setf (get uuv_control_msgs::GoToResponse :definition-)
      "














string LIPB='lipb'
string CUBIC='cubic'
string DUBINS='dubins'
string LINEAR='linear'

uuv_control_msgs/Waypoint waypoint
float64 max_forward_speed
string interpolator

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



(provide :uuv_control_msgs/GoTo "408446fa9ec1d90b38a7053e3dd0ad47")


