;; Auto-generated. Do not edit!


(when (boundp 'uuv_world_ros_plugins_msgs::GetCurrentModel)
  (if (not (find-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
    (make-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
  (shadow 'GetCurrentModel (find-package "UUV_WORLD_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETCURRENTMODEL")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETCURRENTMODEL"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETCURRENTMODELREQUEST")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETCURRENTMODELREQUEST"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETCURRENTMODELRESPONSE")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETCURRENTMODELRESPONSE"))

(in-package "ROS")





(defclass uuv_world_ros_plugins_msgs::GetCurrentModelRequest
  :super ros::object
  :slots ())

(defmethod uuv_world_ros_plugins_msgs::GetCurrentModelRequest
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

(defclass uuv_world_ros_plugins_msgs::GetCurrentModelResponse
  :super ros::object
  :slots (_mean _min _max _noise _mu ))

(defmethod uuv_world_ros_plugins_msgs::GetCurrentModelResponse
  (:init
   (&key
    ((:mean __mean) 0.0)
    ((:min __min) 0.0)
    ((:max __max) 0.0)
    ((:noise __noise) 0.0)
    ((:mu __mu) 0.0)
    )
   (send-super :init)
   (setq _mean (float __mean))
   (setq _min (float __min))
   (setq _max (float __max))
   (setq _noise (float __noise))
   (setq _mu (float __mu))
   self)
  (:mean
   (&optional __mean)
   (if __mean (setq _mean __mean)) _mean)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:max
   (&optional __max)
   (if __max (setq _max __max)) _max)
  (:noise
   (&optional __noise)
   (if __noise (setq _noise __noise)) _noise)
  (:mu
   (&optional __mu)
   (if __mu (setq _mu __mu)) _mu)
  (:serialization-length
   ()
   (+
    ;; float64 _mean
    8
    ;; float64 _min
    8
    ;; float64 _max
    8
    ;; float64 _noise
    8
    ;; float64 _mu
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _mean
       (sys::poke _mean (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _min
       (sys::poke _min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max
       (sys::poke _max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _noise
       (sys::poke _noise (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mu
       (sys::poke _mu (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _mean
     (setq _mean (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _min
     (setq _min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max
     (setq _max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _noise
     (setq _noise (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mu
     (setq _mu (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_world_ros_plugins_msgs::GetCurrentModel
  :super ros::object
  :slots ())

(setf (get uuv_world_ros_plugins_msgs::GetCurrentModel :md5sum-) "b8222571af4e4180b9b706d1e17ad7e1")
(setf (get uuv_world_ros_plugins_msgs::GetCurrentModel :datatype-) "uuv_world_ros_plugins_msgs/GetCurrentModel")
(setf (get uuv_world_ros_plugins_msgs::GetCurrentModel :request) uuv_world_ros_plugins_msgs::GetCurrentModelRequest)
(setf (get uuv_world_ros_plugins_msgs::GetCurrentModel :response) uuv_world_ros_plugins_msgs::GetCurrentModelResponse)

(defmethod uuv_world_ros_plugins_msgs::GetCurrentModelRequest
  (:response () (instance uuv_world_ros_plugins_msgs::GetCurrentModelResponse :init)))

(setf (get uuv_world_ros_plugins_msgs::GetCurrentModelRequest :md5sum-) "b8222571af4e4180b9b706d1e17ad7e1")
(setf (get uuv_world_ros_plugins_msgs::GetCurrentModelRequest :datatype-) "uuv_world_ros_plugins_msgs/GetCurrentModelRequest")
(setf (get uuv_world_ros_plugins_msgs::GetCurrentModelRequest :definition-)
      "














---
float64 mean
float64 min
float64 max
float64 noise
float64 mu

")

(setf (get uuv_world_ros_plugins_msgs::GetCurrentModelResponse :md5sum-) "b8222571af4e4180b9b706d1e17ad7e1")
(setf (get uuv_world_ros_plugins_msgs::GetCurrentModelResponse :datatype-) "uuv_world_ros_plugins_msgs/GetCurrentModelResponse")
(setf (get uuv_world_ros_plugins_msgs::GetCurrentModelResponse :definition-)
      "














---
float64 mean
float64 min
float64 max
float64 noise
float64 mu

")



(provide :uuv_world_ros_plugins_msgs/GetCurrentModel "b8222571af4e4180b9b706d1e17ad7e1")


