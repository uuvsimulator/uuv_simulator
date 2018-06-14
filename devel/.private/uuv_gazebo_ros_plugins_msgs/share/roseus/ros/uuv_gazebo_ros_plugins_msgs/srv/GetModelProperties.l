;; Auto-generated. Do not edit!


(when (boundp 'uuv_gazebo_ros_plugins_msgs::GetModelProperties)
  (if (not (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
    (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
  (shadow 'GetModelProperties (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETMODELPROPERTIES")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETMODELPROPERTIES"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETMODELPROPERTIESREQUEST")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETMODELPROPERTIESREQUEST"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETMODELPROPERTIESRESPONSE")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETMODELPROPERTIESRESPONSE"))

(in-package "ROS")





(defclass uuv_gazebo_ros_plugins_msgs::GetModelPropertiesRequest
  :super ros::object
  :slots ())

(defmethod uuv_gazebo_ros_plugins_msgs::GetModelPropertiesRequest
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

(defclass uuv_gazebo_ros_plugins_msgs::GetModelPropertiesResponse
  :super ros::object
  :slots (_link_names _models ))

(defmethod uuv_gazebo_ros_plugins_msgs::GetModelPropertiesResponse
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:models __models) (let (r) (dotimes (i 0) (push (instance uuv_gazebo_ros_plugins_msgs::UnderwaterObjectModel :init) r)) r))
    )
   (send-super :init)
   (setq _link_names __link_names)
   (setq _models __models)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:models
   (&rest __models)
   (if (keywordp (car __models))
       (send* _models __models)
     (progn
       (if __models (setq _models (car __models)))
       _models)))
  (:serialization-length
   ()
   (+
    ;; string[] _link_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _link_names)) 4
    ;; uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel[] _models
    (apply #'+ (send-all _models :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _link_names
     (write-long (length _link_names) s)
     (dolist (elem _link_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel[] _models
     (write-long (length _models) s)
     (dolist (elem _models)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _link_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _link_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _link_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel[] _models
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _models (let (r) (dotimes (i n) (push (instance uuv_gazebo_ros_plugins_msgs::UnderwaterObjectModel :init) r)) r))
     (dolist (elem- _models)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass uuv_gazebo_ros_plugins_msgs::GetModelProperties
  :super ros::object
  :slots ())

(setf (get uuv_gazebo_ros_plugins_msgs::GetModelProperties :md5sum-) "222d64ab6fa46c24f1abd065170ebe7a")
(setf (get uuv_gazebo_ros_plugins_msgs::GetModelProperties :datatype-) "uuv_gazebo_ros_plugins_msgs/GetModelProperties")
(setf (get uuv_gazebo_ros_plugins_msgs::GetModelProperties :request) uuv_gazebo_ros_plugins_msgs::GetModelPropertiesRequest)
(setf (get uuv_gazebo_ros_plugins_msgs::GetModelProperties :response) uuv_gazebo_ros_plugins_msgs::GetModelPropertiesResponse)

(defmethod uuv_gazebo_ros_plugins_msgs::GetModelPropertiesRequest
  (:response () (instance uuv_gazebo_ros_plugins_msgs::GetModelPropertiesResponse :init)))

(setf (get uuv_gazebo_ros_plugins_msgs::GetModelPropertiesRequest :md5sum-) "222d64ab6fa46c24f1abd065170ebe7a")
(setf (get uuv_gazebo_ros_plugins_msgs::GetModelPropertiesRequest :datatype-) "uuv_gazebo_ros_plugins_msgs/GetModelPropertiesRequest")
(setf (get uuv_gazebo_ros_plugins_msgs::GetModelPropertiesRequest :definition-)
      "














---
string[]  link_names
uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel[] models


================================================================================
MSG: uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel
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

(setf (get uuv_gazebo_ros_plugins_msgs::GetModelPropertiesResponse :md5sum-) "222d64ab6fa46c24f1abd065170ebe7a")
(setf (get uuv_gazebo_ros_plugins_msgs::GetModelPropertiesResponse :datatype-) "uuv_gazebo_ros_plugins_msgs/GetModelPropertiesResponse")
(setf (get uuv_gazebo_ros_plugins_msgs::GetModelPropertiesResponse :definition-)
      "














---
string[]  link_names
uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel[] models


================================================================================
MSG: uuv_gazebo_ros_plugins_msgs/UnderwaterObjectModel
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



(provide :uuv_gazebo_ros_plugins_msgs/GetModelProperties "222d64ab6fa46c24f1abd065170ebe7a")


