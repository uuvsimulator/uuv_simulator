;; Auto-generated. Do not edit!


(when (boundp 'uuv_world_ros_plugins_msgs::TransformToSphericalCoord)
  (if (not (find-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
    (make-package "UUV_WORLD_ROS_PLUGINS_MSGS"))
  (shadow 'TransformToSphericalCoord (find-package "UUV_WORLD_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMTOSPHERICALCOORD")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMTOSPHERICALCOORD"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMTOSPHERICALCOORDREQUEST")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMTOSPHERICALCOORDREQUEST"))
(unless (find-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMTOSPHERICALCOORDRESPONSE")
  (make-package "UUV_WORLD_ROS_PLUGINS_MSGS::TRANSFORMTOSPHERICALCOORDRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass uuv_world_ros_plugins_msgs::TransformToSphericalCoordRequest
  :super ros::object
  :slots (_input ))

(defmethod uuv_world_ros_plugins_msgs::TransformToSphericalCoordRequest
  (:init
   (&key
    ((:input __input) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass uuv_world_ros_plugins_msgs::TransformToSphericalCoordResponse
  :super ros::object
  :slots (_latitude_deg _longitude_deg _altitude ))

(defmethod uuv_world_ros_plugins_msgs::TransformToSphericalCoordResponse
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

(defclass uuv_world_ros_plugins_msgs::TransformToSphericalCoord
  :super ros::object
  :slots ())

(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoord :md5sum-) "5e63b61b1b56d2a5259cc93964944e7b")
(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoord :datatype-) "uuv_world_ros_plugins_msgs/TransformToSphericalCoord")
(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoord :request) uuv_world_ros_plugins_msgs::TransformToSphericalCoordRequest)
(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoord :response) uuv_world_ros_plugins_msgs::TransformToSphericalCoordResponse)

(defmethod uuv_world_ros_plugins_msgs::TransformToSphericalCoordRequest
  (:response () (instance uuv_world_ros_plugins_msgs::TransformToSphericalCoordResponse :init)))

(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoordRequest :md5sum-) "5e63b61b1b56d2a5259cc93964944e7b")
(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoordRequest :datatype-) "uuv_world_ros_plugins_msgs/TransformToSphericalCoordRequest")
(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoordRequest :definition-)
      "














geometry_msgs/Vector3 input

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
---

float64 latitude_deg

float64 longitude_deg

float64 altitude

")

(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoordResponse :md5sum-) "5e63b61b1b56d2a5259cc93964944e7b")
(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoordResponse :datatype-) "uuv_world_ros_plugins_msgs/TransformToSphericalCoordResponse")
(setf (get uuv_world_ros_plugins_msgs::TransformToSphericalCoordResponse :definition-)
      "














geometry_msgs/Vector3 input

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
---

float64 latitude_deg

float64 longitude_deg

float64 altitude

")



(provide :uuv_world_ros_plugins_msgs/TransformToSphericalCoord "5e63b61b1b56d2a5259cc93964944e7b")


