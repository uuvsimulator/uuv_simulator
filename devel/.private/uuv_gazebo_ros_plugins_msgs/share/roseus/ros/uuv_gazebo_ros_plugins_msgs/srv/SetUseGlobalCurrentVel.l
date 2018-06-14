;; Auto-generated. Do not edit!


(when (boundp 'uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel)
  (if (not (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
    (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
  (shadow 'SetUseGlobalCurrentVel (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETUSEGLOBALCURRENTVEL")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETUSEGLOBALCURRENTVEL"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETUSEGLOBALCURRENTVELREQUEST")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETUSEGLOBALCURRENTVELREQUEST"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETUSEGLOBALCURRENTVELRESPONSE")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETUSEGLOBALCURRENTVELRESPONSE"))

(in-package "ROS")





(defclass uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelRequest
  :super ros::object
  :slots (_use_global ))

(defmethod uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelRequest
  (:init
   (&key
    ((:use_global __use_global) nil)
    )
   (send-super :init)
   (setq _use_global __use_global)
   self)
  (:use_global
   (&optional __use_global)
   (if __use_global (setq _use_global __use_global)) _use_global)
  (:serialization-length
   ()
   (+
    ;; bool _use_global
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _use_global
       (if _use_global (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _use_global
     (setq _use_global (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelResponse
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

(defclass uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel
  :super ros::object
  :slots ())

(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel :md5sum-) "02d40f951486d0b4bee34e7b1c66f745")
(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel :datatype-) "uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVel")
(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel :request) uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelRequest)
(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVel :response) uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelResponse)

(defmethod uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelRequest
  (:response () (instance uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelResponse :init)))

(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelRequest :md5sum-) "02d40f951486d0b4bee34e7b1c66f745")
(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelRequest :datatype-) "uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVelRequest")
(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelRequest :definition-)
      "














bool use_global
---
bool success

")

(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelResponse :md5sum-) "02d40f951486d0b4bee34e7b1c66f745")
(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelResponse :datatype-) "uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVelResponse")
(setf (get uuv_gazebo_ros_plugins_msgs::SetUseGlobalCurrentVelResponse :definition-)
      "














bool use_global
---
bool success

")



(provide :uuv_gazebo_ros_plugins_msgs/SetUseGlobalCurrentVel "02d40f951486d0b4bee34e7b1c66f745")


