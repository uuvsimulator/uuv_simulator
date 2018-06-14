;; Auto-generated. Do not edit!


(when (boundp 'uuv_world_ros_plugins_msgs::SetCurrentVelocity)
  (if (not (find-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
    (make-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
  (shadow 'SetCurrentVelocity (find-package "UUV_WORLD_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTVELOCITY")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTVELOCITY"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTVELOCITYREQUEST")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTVELOCITYREQUEST"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTVELOCITYRESPONSE")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTVELOCITYRESPONSE"))

(in-package "ROS")





(defclass uuv_world_ros_plugins_msgs::SetCurrentVelocityRequest
  :super ros::object
  :slots (_velocity _horizontal_angle _vertical_angle ))

(defmethod uuv_world_ros_plugins_msgs::SetCurrentVelocityRequest
  (:init
   (&key
    ((:velocity __velocity) 0.0)
    ((:horizontal_angle __horizontal_angle) 0.0)
    ((:vertical_angle __vertical_angle) 0.0)
    )
   (send-super :init)
   (setq _velocity (float __velocity))
   (setq _horizontal_angle (float __horizontal_angle))
   (setq _vertical_angle (float __vertical_angle))
   self)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:horizontal_angle
   (&optional __horizontal_angle)
   (if __horizontal_angle (setq _horizontal_angle __horizontal_angle)) _horizontal_angle)
  (:vertical_angle
   (&optional __vertical_angle)
   (if __vertical_angle (setq _vertical_angle __vertical_angle)) _vertical_angle)
  (:serialization-length
   ()
   (+
    ;; float64 _velocity
    8
    ;; float64 _horizontal_angle
    8
    ;; float64 _vertical_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _horizontal_angle
       (sys::poke _horizontal_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vertical_angle
       (sys::poke _vertical_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _horizontal_angle
     (setq _horizontal_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vertical_angle
     (setq _vertical_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_world_ros_plugins_msgs::SetCurrentVelocityResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_world_ros_plugins_msgs::SetCurrentVelocityResponse
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

(defclass uuv_world_ros_plugins_msgs::SetCurrentVelocity
  :super ros::object
  :slots ())

(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocity :md5sum-) "3389770cff5466e5c98d6200f7909bd7")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocity :datatype-) "uuv_world_ros_plugins_msgs/SetCurrentVelocity")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocity :request) uuv_world_ros_plugins_msgs::SetCurrentVelocityRequest)
(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocity :response) uuv_world_ros_plugins_msgs::SetCurrentVelocityResponse)

(defmethod uuv_world_ros_plugins_msgs::SetCurrentVelocityRequest
  (:response () (instance uuv_world_ros_plugins_msgs::SetCurrentVelocityResponse :init)))

(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocityRequest :md5sum-) "3389770cff5466e5c98d6200f7909bd7")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocityRequest :datatype-) "uuv_world_ros_plugins_msgs/SetCurrentVelocityRequest")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocityRequest :definition-)
      "














float64 velocity
float64 horizontal_angle
float64 vertical_angle
---
bool success

")

(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocityResponse :md5sum-) "3389770cff5466e5c98d6200f7909bd7")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocityResponse :datatype-) "uuv_world_ros_plugins_msgs/SetCurrentVelocityResponse")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentVelocityResponse :definition-)
      "














float64 velocity
float64 horizontal_angle
float64 vertical_angle
---
bool success

")



(provide :uuv_world_ros_plugins_msgs/SetCurrentVelocity "3389770cff5466e5c98d6200f7909bd7")


