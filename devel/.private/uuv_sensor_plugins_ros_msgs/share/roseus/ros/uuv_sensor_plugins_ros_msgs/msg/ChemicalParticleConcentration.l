;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_plugins_ros_msgs::ChemicalParticleConcentration)
  (if (not (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
    (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
  (shadow 'ChemicalParticleConcentration (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS")))
(unless (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::CHEMICALPARTICLECONCENTRATION")
  (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS::CHEMICALPARTICLECONCENTRATION"))

(in-package "ROS")
;;//! \htmlinclude ChemicalParticleConcentration.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uuv_sensor_plugins_ros_msgs::ChemicalParticleConcentration
  :super ros::object
  :slots (_header _position _latitude _longitude _depth _concentration _is_measuring ))

(defmethod uuv_sensor_plugins_ros_msgs::ChemicalParticleConcentration
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:depth __depth) 0.0)
    ((:concentration __concentration) 0.0)
    ((:is_measuring __is_measuring) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _depth (float __depth))
   (setq _concentration (float __concentration))
   (setq _is_measuring __is_measuring)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:concentration
   (&optional __concentration)
   (if __concentration (setq _concentration __concentration)) _concentration)
  (:is_measuring
   (&optional __is_measuring)
   (if __is_measuring (setq _is_measuring __is_measuring)) _is_measuring)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _depth
    8
    ;; float64 _concentration
    8
    ;; bool _is_measuring
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _concentration
       (sys::poke _concentration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _is_measuring
       (if _is_measuring (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _depth
     (setq _depth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _concentration
     (setq _concentration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _is_measuring
     (setq _is_measuring (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get uuv_sensor_plugins_ros_msgs::ChemicalParticleConcentration :md5sum-) "f17ac73d7dd3916a4fd6f721f0ccbe37")
(setf (get uuv_sensor_plugins_ros_msgs::ChemicalParticleConcentration :datatype-) "uuv_sensor_plugins_ros_msgs/ChemicalParticleConcentration")
(setf (get uuv_sensor_plugins_ros_msgs::ChemicalParticleConcentration :definition-)
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
# Sensor link position
geometry_msgs/Vector3 position
# Latitude in degrees (WGS84 coordinates)
float64 latitude
# Longiture in degrees (WGS84 coordinates)
float64 longitude
# Depth in meters (WGS84 coordinates)
float64 depth
# Particle concentration
float64 concentration
# Status, false if no plume particles are received
bool is_measuring

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



(provide :uuv_sensor_plugins_ros_msgs/ChemicalParticleConcentration "f17ac73d7dd3916a4fd6f721f0ccbe37")


