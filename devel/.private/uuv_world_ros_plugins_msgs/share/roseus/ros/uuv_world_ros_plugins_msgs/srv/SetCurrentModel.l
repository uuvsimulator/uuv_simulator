;; Auto-generated. Do not edit!


(when (boundp 'uuv_world_ros_plugins_msgs::SetCurrentModel)
  (if (not (find-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
    (make-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
  (shadow 'SetCurrentModel (find-package "UUV_WORLD_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTMODEL")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTMODEL"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTMODELREQUEST")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTMODELREQUEST"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTMODELRESPONSE")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETCURRENTMODELRESPONSE"))

(in-package "ROS")





(defclass uuv_world_ros_plugins_msgs::SetCurrentModelRequest
  :super ros::object
  :slots (_mean _min _max _noise _mu ))

(defmethod uuv_world_ros_plugins_msgs::SetCurrentModelRequest
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

(defclass uuv_world_ros_plugins_msgs::SetCurrentModelResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_world_ros_plugins_msgs::SetCurrentModelResponse
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

(defclass uuv_world_ros_plugins_msgs::SetCurrentModel
  :super ros::object
  :slots ())

(setf (get uuv_world_ros_plugins_msgs::SetCurrentModel :md5sum-) "97b431fd7a0d7100472976b98c315e14")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentModel :datatype-) "uuv_world_ros_plugins_msgs/SetCurrentModel")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentModel :request) uuv_world_ros_plugins_msgs::SetCurrentModelRequest)
(setf (get uuv_world_ros_plugins_msgs::SetCurrentModel :response) uuv_world_ros_plugins_msgs::SetCurrentModelResponse)

(defmethod uuv_world_ros_plugins_msgs::SetCurrentModelRequest
  (:response () (instance uuv_world_ros_plugins_msgs::SetCurrentModelResponse :init)))

(setf (get uuv_world_ros_plugins_msgs::SetCurrentModelRequest :md5sum-) "97b431fd7a0d7100472976b98c315e14")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentModelRequest :datatype-) "uuv_world_ros_plugins_msgs/SetCurrentModelRequest")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentModelRequest :definition-)
      "














float64 mean
float64 min
float64 max
float64 noise
float64 mu
---
bool success

")

(setf (get uuv_world_ros_plugins_msgs::SetCurrentModelResponse :md5sum-) "97b431fd7a0d7100472976b98c315e14")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentModelResponse :datatype-) "uuv_world_ros_plugins_msgs/SetCurrentModelResponse")
(setf (get uuv_world_ros_plugins_msgs::SetCurrentModelResponse :definition-)
      "














float64 mean
float64 min
float64 max
float64 noise
float64 mu
---
bool success

")



(provide :uuv_world_ros_plugins_msgs/SetCurrentModel "97b431fd7a0d7100472976b98c315e14")


