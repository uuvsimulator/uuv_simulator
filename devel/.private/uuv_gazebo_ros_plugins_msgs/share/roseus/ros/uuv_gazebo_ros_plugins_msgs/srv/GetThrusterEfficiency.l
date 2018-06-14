;; Auto-generated. Do not edit!


(when (boundp 'uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency)
  (if (not (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
    (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
  (shadow 'GetThrusterEfficiency (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTEREFFICIENCY")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTEREFFICIENCY"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTEREFFICIENCYREQUEST")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTEREFFICIENCYREQUEST"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTEREFFICIENCYRESPONSE")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::GETTHRUSTEREFFICIENCYRESPONSE"))

(in-package "ROS")





(defclass uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyRequest
  :super ros::object
  :slots ())

(defmethod uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyRequest
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

(defclass uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyResponse
  :super ros::object
  :slots (_efficiency ))

(defmethod uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyResponse
  (:init
   (&key
    ((:efficiency __efficiency) 0.0)
    )
   (send-super :init)
   (setq _efficiency (float __efficiency))
   self)
  (:efficiency
   (&optional __efficiency)
   (if __efficiency (setq _efficiency __efficiency)) _efficiency)
  (:serialization-length
   ()
   (+
    ;; float64 _efficiency
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _efficiency
       (sys::poke _efficiency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _efficiency
     (setq _efficiency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency
  :super ros::object
  :slots ())

(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency :md5sum-) "b80ec71e671b93e4cc403df1ac4c8a86")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency :datatype-) "uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiency")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency :request) uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyRequest)
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiency :response) uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyResponse)

(defmethod uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyRequest
  (:response () (instance uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyResponse :init)))

(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyRequest :md5sum-) "b80ec71e671b93e4cc403df1ac4c8a86")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyRequest :datatype-) "uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiencyRequest")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyRequest :definition-)
      "














---
float64 efficiency

")

(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyResponse :md5sum-) "b80ec71e671b93e4cc403df1ac4c8a86")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyResponse :datatype-) "uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiencyResponse")
(setf (get uuv_gazebo_ros_plugins_msgs::GetThrusterEfficiencyResponse :definition-)
      "














---
float64 efficiency

")



(provide :uuv_gazebo_ros_plugins_msgs/GetThrusterEfficiency "b80ec71e671b93e4cc403df1ac4c8a86")


