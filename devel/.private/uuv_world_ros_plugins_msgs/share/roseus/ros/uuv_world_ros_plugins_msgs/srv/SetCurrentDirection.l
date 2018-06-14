;; Auto-generated. Do not edit!


(when (boundp 'uuv_world_ros_plugins_msgs::SetCurrentDirection)
  (if (not (find-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
    (make-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
  (shadow 'SetCurrentDirection (find-package "UUV_WORLD_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTDIRECTION")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTDIRECTION"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTDIRECTIONREQUEST")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTDIRECTIONREQUEST"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTDIRECTIONRESPONSE")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTDIRECTIONRESPONSE"))

(in-package "ROS")





(defclass uuv_world_ros_plugins_msgs::SetCurrentDirectionRequest
  :super ros::object
  :slots (_angle ))

(defmethod uuv_world_ros_plugins_msgs::SetCurrentDirectionRequest
  (:init
   (&key
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_world_ros_plugins_msgs::SetCurrentDirectionResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_world_ros_plugins_msgs::SetCurrentDirectionResponse
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

(defclass uuv_world_ros_plugins_msgs::SetCurrentDirection
  :super ros::object
  :slots ())

(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirection :md5sum-) "c1a76fcaf62dc4534903e93216b59a79")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirection :datatype-) "uuv_world_ros_plugins_msgs/SetCurrentDirection")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirection :request) uuv_world_ros_plugins_msgs::SetCurrentDirectionRequest)
(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirection :response) uuv_world_ros_plugins_msgs::SetCurrentDirectionResponse)

(defmethod uuv_world_ros_plugins_msgs::SetCurrentDirectionRequest
  (:response () (instance uuv_world_ros_plugins_msgs::SetCurrentDirectionResponse :init)))

(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirectionRequest :md5sum-) "c1a76fcaf62dc4534903e93216b59a79")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirectionRequest :datatype-) "uuv_world_ros_plugins_msgs/SetCurrentDirectionRequest")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirectionRequest :definition-)
      "














float64 angle
---
bool success

")

(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirectionResponse :md5sum-) "c1a76fcaf62dc4534903e93216b59a79")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirectionResponse :datatype-) "uuv_world_ros_plugins_msgs/SetCurrentDirectionResponse")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentDirectionResponse :definition-)
      "














float64 angle
---
bool success

")



(provide :uuv_world_ros_plugins_msgs/SetCurrentDirection "c1a76fcaf62dc4534903e93216b59a79")


