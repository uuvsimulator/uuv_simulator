;; Auto-generated. Do not edit!


(when (boundp 'uuv_gazebo_ros_plugins_msgs::UnderwaterObjectModel)
  (if (not (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
    (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
  (shadow 'UnderwaterObjectModel (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::UNDERWATEROBJECTMODEL")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::UNDERWATEROBJECTMODEL"))

(in-package "ROS")
;;//! \htmlinclude UnderwaterObjectModel.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass uuv_gazebo_ros_plugins_msgs::UnderwaterObjectModel
  :super ros::object
  :slots (_added_mass _linear_damping _linear_damping_forward_speed _quadratic_damping _volume _bbox_height _bbox_length _bbox_width _fluid_density _cob _neutrally_buoyant _inertia ))

(defmethod uuv_gazebo_ros_plugins_msgs::UnderwaterObjectModel
  (:init
   (&key
    ((:added_mass __added_mass) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:linear_damping __linear_damping) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:linear_damping_forward_speed __linear_damping_forward_speed) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:quadratic_damping __quadratic_damping) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:volume __volume) 0.0)
    ((:bbox_height __bbox_height) 0.0)
    ((:bbox_length __bbox_length) 0.0)
    ((:bbox_width __bbox_width) 0.0)
    ((:fluid_density __fluid_density) 0.0)
    ((:cob __cob) (instance geometry_msgs::Vector3 :init))
    ((:neutrally_buoyant __neutrally_buoyant) nil)
    ((:inertia __inertia) (instance geometry_msgs::Inertia :init))
    )
   (send-super :init)
   (setq _added_mass __added_mass)
   (setq _linear_damping __linear_damping)
   (setq _linear_damping_forward_speed __linear_damping_forward_speed)
   (setq _quadratic_damping __quadratic_damping)
   (setq _volume (float __volume))
   (setq _bbox_height (float __bbox_height))
   (setq _bbox_length (float __bbox_length))
   (setq _bbox_width (float __bbox_width))
   (setq _fluid_density (float __fluid_density))
   (setq _cob __cob)
   (setq _neutrally_buoyant __neutrally_buoyant)
   (setq _inertia __inertia)
   self)
  (:added_mass
   (&optional __added_mass)
   (if __added_mass (setq _added_mass __added_mass)) _added_mass)
  (:linear_damping
   (&optional __linear_damping)
   (if __linear_damping (setq _linear_damping __linear_damping)) _linear_damping)
  (:linear_damping_forward_speed
   (&optional __linear_damping_forward_speed)
   (if __linear_damping_forward_speed (setq _linear_damping_forward_speed __linear_damping_forward_speed)) _linear_damping_forward_speed)
  (:quadratic_damping
   (&optional __quadratic_damping)
   (if __quadratic_damping (setq _quadratic_damping __quadratic_damping)) _quadratic_damping)
  (:volume
   (&optional __volume)
   (if __volume (setq _volume __volume)) _volume)
  (:bbox_height
   (&optional __bbox_height)
   (if __bbox_height (setq _bbox_height __bbox_height)) _bbox_height)
  (:bbox_length
   (&optional __bbox_length)
   (if __bbox_length (setq _bbox_length __bbox_length)) _bbox_length)
  (:bbox_width
   (&optional __bbox_width)
   (if __bbox_width (setq _bbox_width __bbox_width)) _bbox_width)
  (:fluid_density
   (&optional __fluid_density)
   (if __fluid_density (setq _fluid_density __fluid_density)) _fluid_density)
  (:cob
   (&rest __cob)
   (if (keywordp (car __cob))
       (send* _cob __cob)
     (progn
       (if __cob (setq _cob (car __cob)))
       _cob)))
  (:neutrally_buoyant
   (&optional __neutrally_buoyant)
   (if __neutrally_buoyant (setq _neutrally_buoyant __neutrally_buoyant)) _neutrally_buoyant)
  (:inertia
   (&rest __inertia)
   (if (keywordp (car __inertia))
       (send* _inertia __inertia)
     (progn
       (if __inertia (setq _inertia (car __inertia)))
       _inertia)))
  (:serialization-length
   ()
   (+
    ;; float64[] _added_mass
    (* 8    (length _added_mass)) 4
    ;; float64[] _linear_damping
    (* 8    (length _linear_damping)) 4
    ;; float64[] _linear_damping_forward_speed
    (* 8    (length _linear_damping_forward_speed)) 4
    ;; float64[] _quadratic_damping
    (* 8    (length _quadratic_damping)) 4
    ;; float64 _volume
    8
    ;; float64 _bbox_height
    8
    ;; float64 _bbox_length
    8
    ;; float64 _bbox_width
    8
    ;; float64 _fluid_density
    8
    ;; geometry_msgs/Vector3 _cob
    (send _cob :serialization-length)
    ;; bool _neutrally_buoyant
    1
    ;; geometry_msgs/Inertia _inertia
    (send _inertia :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _added_mass
     (write-long (length _added_mass) s)
     (dotimes (i (length _added_mass))
       (sys::poke (elt _added_mass i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _linear_damping
     (write-long (length _linear_damping) s)
     (dotimes (i (length _linear_damping))
       (sys::poke (elt _linear_damping i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _linear_damping_forward_speed
     (write-long (length _linear_damping_forward_speed) s)
     (dotimes (i (length _linear_damping_forward_speed))
       (sys::poke (elt _linear_damping_forward_speed i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _quadratic_damping
     (write-long (length _quadratic_damping) s)
     (dotimes (i (length _quadratic_damping))
       (sys::poke (elt _quadratic_damping i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _volume
       (sys::poke _volume (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bbox_height
       (sys::poke _bbox_height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bbox_length
       (sys::poke _bbox_length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bbox_width
       (sys::poke _bbox_width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _fluid_density
       (sys::poke _fluid_density (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Vector3 _cob
       (send _cob :serialize s)
     ;; bool _neutrally_buoyant
       (if _neutrally_buoyant (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Inertia _inertia
       (send _inertia :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _added_mass
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _added_mass (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _added_mass i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _linear_damping
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _linear_damping (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _linear_damping i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _linear_damping_forward_speed
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _linear_damping_forward_speed (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _linear_damping_forward_speed i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _quadratic_damping
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _quadratic_damping (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _quadratic_damping i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _volume
     (setq _volume (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bbox_height
     (setq _bbox_height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bbox_length
     (setq _bbox_length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bbox_width
     (setq _bbox_width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _fluid_density
     (setq _fluid_density (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Vector3 _cob
     (send _cob :deserialize buf ptr-) (incf ptr- (send _cob :serialization-length))
   ;; bool _neutrally_buoyant
     (setq _neutrally_buoyant (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Inertia _inertia
     (send _inertia :deserialize buf ptr-) (incf ptr- (send _inertia :serialization-length))
   ;;
   self)
  )

(setf (get uuv_gazebo_ros_plugins_msgs::UnderwaterObjectModel :md5sum-) "35ada57addb7202af96020ea2ddc109c")
(setf (get uuv_gazebo_ros_plugins_msgs::UnderwaterObjectModel :datatype-) "uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel")
(setf (get uuv_gazebo_ros_plugins_msgs::UnderwaterObjectModel :definition-)
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

float64[] added_mass
float64[] linear_damping
float64[] linear_damping_forward_speed
float64[] quadratic_damping
float64 volume
float64 bbox_height
float64 bbox_length
float64 bbox_width
float64 fluid_density
geometry_msgs/Vector3 cob
bool neutrally_buoyant
geometry_msgs/Inertia inertia
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
================================================================================
MSG: geometry_msgs/Inertia
# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz

")



(provide :uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel "35ada57addb7202af96020ea2ddc109c")


