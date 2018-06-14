;; Auto-generated. Do not edit!


(when (boundp 'uuv_gazebo_ros_plugins_msgs::GetThrusterState)
  (if (not (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
    (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
  (shadow 'GetThrusterState (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTERSTATE")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTERSTATE"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTERSTATEREQUEST")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTERSTATEREQUEST"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTERSTATERESPONSE")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTERSTATERESPONSE"))

(in-package "ROS")





(defclass uuv_gazebo_ros_plugins_msgs::GetThrusterStateRequest
  :super ros::object
  :slots ())

(defmethod uuv_gazebo_ros_plugins_msgs::GetThrusterStateRequest
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

(defclass uuv_gazebo_ros_plugins_msgs::GetThrusterStateResponse
  :super ros::object
  :slots (_is_on ))

(defmethod uuv_gazebo_ros_plugins_msgs::GetThrusterStateResponse
  (:init
   (&key
    ((:is_on __is_on) nil)
    )
   (send-super :init)
   (setq _is_on __is_on)
   self)
  (:is_on
   (&optional __is_on)
   (if __is_on (setq _is_on __is_on)) _is_on)
  (:serialization-length
   ()
   (+
    ;; bool _is_on
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_on
       (if _is_on (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_on
     (setq _is_on (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uuv_gazebo_ros_plugins_msgs::GetThrusterState
  :super ros::object
  :slots ())

(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterState :md5sum-) "e2fdda8431274beee70eebaa081c813e")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterState :datatype-) "uuv_gazebo_ros_plugins_msgs/GetThrusterState")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterState :request) uuv_gazebo_ros_plugins_msgs::GetThrusterStateRequest)
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterState :response) uuv_gazebo_ros_plugins_msgs::GetThrusterStateResponse)

(defmethod uuv_gazebo_ros_plugins_msgs::GetThrusterStateRequest
  (:response () (instance uuv_gazebo_ros_plugins_msgs::GetThrusterStateResponse :init)))

(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterStateRequest :md5sum-) "e2fdda8431274beee70eebaa081c813e")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterStateRequest :datatype-) "uuv_gazebo_ros_plugins_msgs/GetThrusterStateRequest")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterStateRequest :definition-)
      "














---
bool is_on

")

(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterStateResponse :md5sum-) "e2fdda8431274beee70eebaa081c813e")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterStateResponse :datatype-) "uuv_gazebo_ros_plugins_msgs/GetThrusterStateResponse")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterStateResponse :definition-)
      "














---
bool is_on

")



(provide :uuv_gazebo_ros_plugins_msgs/GetThrusterState "e2fdda8431274beee70eebaa081c813e")


