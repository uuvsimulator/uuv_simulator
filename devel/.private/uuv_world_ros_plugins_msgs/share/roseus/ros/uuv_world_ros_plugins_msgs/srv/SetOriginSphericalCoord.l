;; Auto-generated. Do not edit!


(when (boundp 'uuv_world_ros_plugins_msgs::SetOriginSphericalCoord)
  (if (not (find-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
    (make-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
  (shadow 'SetOriginSphericalCoord (find-package "UUV_WORLD_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETORIGINSPHERICALCOORD")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETORIGINSPHERICALCOORD"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETORIGINSPHERICALCOORDREQUEST")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETORIGINSPHERICALCOORDREQUEST"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETORIGINSPHERICALCOORDRESPONSE")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::SETORIGINSPHERICALCOORDRESPONSE"))

(in-package "ROS")





(defclass uuv_world_ros_plugins_msgs::SetOriginSphericalCoordRequest
  :super ros::object
  :slots (_latitude_deg _longitude_deg _altitude ))

(defmethod uuv_world_ros_plugins_msgs::SetOriginSphericalCoordRequest
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

(defclass uuv_world_ros_plugins_msgs::SetOriginSphericalCoordResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_world_ros_plugins_msgs::SetOriginSphericalCoordResponse
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

(defclass uuv_world_ros_plugins_msgs::SetOriginSphericalCoord
  :super ros::object
  :slots ())

(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoord :md5sum-) "be1cd7093c79a14933c2ac116d54917a")
(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoord :datatype-) "uuv_world_ros_plugins_msgs/SetOriginSphericalCoord")
(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoord :request) uuv_world_ros_plugins_msgs::SetOriginSphericalCoordRequest)
(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoord :response) uuv_world_ros_plugins_msgs::SetOriginSphericalCoordResponse)

(defmethod uuv_world_ros_plugins_msgs::SetOriginSphericalCoordRequest
  (:response () (instance uuv_world_ros_plugins_msgs::SetOriginSphericalCoordResponse :init)))

(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoordRequest :md5sum-) "be1cd7093c79a14933c2ac116d54917a")
(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoordRequest :datatype-) "uuv_world_ros_plugins_msgs/SetOriginSphericalCoordRequest")
(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoordRequest :definition-)
      "















float64 latitude_deg

float64 longitude_deg

float64 altitude
---
bool success

")

(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoordResponse :md5sum-) "be1cd7093c79a14933c2ac116d54917a")
(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoordResponse :datatype-) "uuv_world_ros_plugins_msgs/SetOriginSphericalCoordResponse")
(setf (get uuv_world_ros_plugins_msgs::SetOriginSphericalCoordResponse :definition-)
      "















float64 latitude_deg

float64 longitude_deg

float64 altitude
---
bool success

")



(provide :uuv_world_ros_plugins_msgs/SetOriginSphericalCoord "be1cd7093c79a14933c2ac116d54917a")


