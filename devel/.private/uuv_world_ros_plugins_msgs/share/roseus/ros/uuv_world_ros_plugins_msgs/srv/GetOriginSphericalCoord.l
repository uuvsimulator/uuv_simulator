;; Auto-generated. Do not edit!


(when (boundp 'uuv_world_ros_plugins_msgs::GetOriginSphericalCoord)
  (if (not (find-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
    (make-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
  (shadow 'GetOriginSphericalCoord (find-package "UUV_WORLD_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETORIGINSPHERICALCOORD")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETORIGINSPHERICALCOORD"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETORIGINSPHERICALCOORDREQUEST")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETORIGINSPHERICALCOORDREQUEST"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETORIGINSPHERICALCOORDRESPONSE")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::GETORIGINSPHERICALCOORDRESPONSE"))

(in-package "ROS")





(defclass uuv_world_ros_plugins_msgs::GetOriginSphericalCoordRequest
  :super ros::object
  :slots ())

(defmethod uuv_world_ros_plugins_msgs::GetOriginSphericalCoordRequest
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

(defclass uuv_world_ros_plugins_msgs::GetOriginSphericalCoordResponse
  :super ros::object
  :slots (_latitude_deg _longitude_deg _altitude ))

(defmethod uuv_world_ros_plugins_msgs::GetOriginSphericalCoordResponse
  (:init
   (&key
    ((:latitude_deg __latitude_deg) 0.0)
    ((:longitude_deg __longitude_deg) 0.0)
    ((:altitude __altitude) 0.0)
    )
   (send-super :init)
   (setq _latitude_deg (float __latitude_deg))
   (setq _longitude_deg (float __longitude_deg))
   (setq _altitude (float __altitude))
   self)
  (:latitude_deg
   (&optional __latitude_deg)
   (if __latitude_deg (setq _latitude_deg __latitude_deg)) _latitude_deg)
  (:longitude_deg
   (&optional __longitude_deg)
   (if __longitude_deg (setq _longitude_deg __longitude_deg)) _longitude_deg)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:serialization-length
   ()
   (+
    ;; float64 _latitude_deg
    8
    ;; float64 _longitude_deg
    8
    ;; float64 _altitude
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _latitude_deg
       (sys::poke _latitude_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude_deg
       (sys::poke _longitude_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _latitude_deg
     (setq _latitude_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude_deg
     (setq _longitude_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_world_ros_plugins_msgs::GetOriginSphericalCoord
  :super ros::object
  :slots ())

(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoord :md5sum-) "60457d630fe21cc5f8f6bd5d0fc90156")
(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoord :datatype-) "uuv_world_ros_plugins_msgs/GetOriginSphericalCoord")
(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoord :request) uuv_world_ros_plugins_msgs::GetOriginSphericalCoordRequest)
(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoord :response) uuv_world_ros_plugins_msgs::GetOriginSphericalCoordResponse)

(defmethod uuv_world_ros_plugins_msgs::GetOriginSphericalCoordRequest
  (:response () (instance uuv_world_ros_plugins_msgs::GetOriginSphericalCoordResponse :init)))

(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoordRequest :md5sum-) "60457d630fe21cc5f8f6bd5d0fc90156")
(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoordRequest :datatype-) "uuv_world_ros_plugins_msgs/GetOriginSphericalCoordRequest")
(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoordRequest :definition-)
      "














---

float64 latitude_deg

float64 longitude_deg

float64 altitude

")

(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoordResponse :md5sum-) "60457d630fe21cc5f8f6bd5d0fc90156")
(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoordResponse :datatype-) "uuv_world_ros_plugins_msgs/GetOriginSphericalCoordResponse")
(setf (get uuv_world_ros_plugins_msgs::GetOriginSphericalCoordResponse :definition-)
      "














---

float64 latitude_deg

float64 longitude_deg

float64 altitude

")



(provide :uuv_world_ros_plugins_msgs/GetOriginSphericalCoord "60457d630fe21cc5f8f6bd5d0fc90156")


