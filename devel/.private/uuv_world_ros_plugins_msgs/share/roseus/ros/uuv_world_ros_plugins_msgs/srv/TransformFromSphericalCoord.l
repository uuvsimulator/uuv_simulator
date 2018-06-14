;; Auto-generated. Do not edit!


(when (boundp 'uuv_world_ros_plugins_msgs::TransformFromSphericalCoord)
  (if (not (find-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
    (make-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
  (shadow 'TransformFromSphericalCoord (find-package "UUV_WORLD_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMFROMSPHERICALCOORD")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMFROMSPHERICALCOORD"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMFROMSPHERICALCOORDREQUEST")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMFROMSPHERICALCOORDREQUEST"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMFROMSPHERICALCOORDRESPONSE")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMFROMSPHERICALCOORDRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass uuv_world_ros_plugins_msgs::TransformFromSphericalCoordRequest
  :super ros::object
  :slots (_latitude_deg _longitude_deg _altitude ))

(defmethod uuv_world_ros_plugins_msgs::TransformFromSphericalCoordRequest
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

(defclass uuv_world_ros_plugins_msgs::TransformFromSphericalCoordResponse
  :super ros::object
  :slots (_output ))

(defmethod uuv_world_ros_plugins_msgs::TransformFromSphericalCoordResponse
  (:init
   (&key
    ((:output __output) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass uuv_world_ros_plugins_msgs::TransformFromSphericalCoord
  :super ros::object
  :slots ())

(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoord :md5sum-) "7a7b547d22a150426bbc278358d5fb7e")
(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoord :datatype-) "uuv_world_ros_plugins_msgs/TransformFromSphericalCoord")
(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoord :request) uuv_world_ros_plugins_msgs::TransformFromSphericalCoordRequest)
(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoord :response) uuv_world_ros_plugins_msgs::TransformFromSphericalCoordResponse)

(defmethod uuv_world_ros_plugins_msgs::TransformFromSphericalCoordRequest
  (:response () (instance uuv_world_ros_plugins_msgs::TransformFromSphericalCoordResponse :init)))

(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoordRequest :md5sum-) "7a7b547d22a150426bbc278358d5fb7e")
(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoordRequest :datatype-) "uuv_world_ros_plugins_msgs/TransformFromSphericalCoordRequest")
(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoordRequest :definition-)
      "















float64 latitude_deg

float64 longitude_deg

float64 altitude
---
geometry_msgs/Vector3 output


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
")

(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoordResponse :md5sum-) "7a7b547d22a150426bbc278358d5fb7e")
(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoordResponse :datatype-) "uuv_world_ros_plugins_msgs/TransformFromSphericalCoordResponse")
(setf (get uuv_world_ros_plugins_msgs::TransformFromSphericalCoordResponse :definition-)
      "















float64 latitude_deg

float64 longitude_deg

float64 altitude
---
geometry_msgs/Vector3 output


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
")



(provide :uuv_world_ros_plugins_msgs/TransformFromSphericalCoord "7a7b547d22a150426bbc278358d5fb7e")


