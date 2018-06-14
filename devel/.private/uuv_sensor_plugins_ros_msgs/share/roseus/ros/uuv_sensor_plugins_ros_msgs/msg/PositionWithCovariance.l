;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_plugins_ros_msgs::PositionWithCovariance)
  (if (not (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
    (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
  (shadow 'PositionWithCovariance (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS")))
(unless (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::POSITIONWITHCOVARIANCE")
  (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS::POSITIONWITHCOVARIANCE"))

(in-package "ROS")
;;//! \htmlinclude PositionWithCovariance.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass uuv_sensor_plugins_ros_msgs::PositionWithCovariance
  :super ros::object
  :slots (_pos _covariance ))

(defmethod uuv_sensor_plugins_ros_msgs::PositionWithCovariance
  (:init
   (&key
    ((:pos __pos) (instance geometry_msgs::Point :init))
    ((:covariance __covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _covariance __covariance)
   self)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:covariance
   (&optional __covariance)
   (if __covariance (setq _covariance __covariance)) _covariance)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _pos
    (send _pos :serialization-length)
    ;; float64[9] _covariance
    (* 8    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _pos
       (send _pos :serialize s)
     ;; float64[9] _covariance
     (dotimes (i 9)
       (sys::poke (elt _covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;; float64[9] _covariance
   (dotimes (i (length _covariance))
     (setf (elt _covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get uuv_sensor_plugins_ros_msgs::PositionWithCovariance :md5sum-) "4a54596b2ea1a0ed659f46ab0b26f202")
(setf (get uuv_sensor_plugins_ros_msgs::PositionWithCovariance :datatype-) "uuv_sensor_plugins_ros_msgs/PositionWithCovariance")
(setf (get uuv_sensor_plugins_ros_msgs::PositionWithCovariance :definition-)
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

# This represents a position in free space with uncertainty.

geometry_msgs/Point pos

# Row-major representation of the 3x3 covariance matrix
float64[9] covariance

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :uuv_sensor_plugins_ros_msgs/PositionWithCovariance "4a54596b2ea1a0ed659f46ab0b26f202")


