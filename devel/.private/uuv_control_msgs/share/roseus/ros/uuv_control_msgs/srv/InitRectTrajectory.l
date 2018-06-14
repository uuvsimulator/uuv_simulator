;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::InitRectTrajectory)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'InitRectTrajectory (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::INITRECTTRAJECTORY")
  (make-package "UUV_CONTROL_MSGS::INITRECTTRAJECTORY"))
(unless (find-package "UUV_CONTROL_MSGS::INITRECTTRAJECTORYREQUEST")
  (make-package "UUV_CONTROL_MSGS::INITRECTTRAJECTORYREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::INITRECTTRAJECTORYRESPONSE")
  (make-package "UUV_CONTROL_MSGS::INITRECTTRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass uuv_control_msgs::InitRectTrajectoryRequest
  :super ros::object
  :slots (_start_time _start_now _origin _height _width _angle_offset _heading_offset _max_forward_speed ))

(defmethod uuv_control_msgs::InitRectTrajectoryRequest
  (:init
   (&key
    ((:start_time __start_time) (instance std_msgs::Time :init))
    ((:start_now __start_now) nil)
    ((:origin __origin) (instance geometry_msgs::Point :init))
    ((:height __height) 0.0)
    ((:width __width) 0.0)
    ((:angle_offset __angle_offset) 0.0)
    ((:heading_offset __heading_offset) 0.0)
    ((:max_forward_speed __max_forward_speed) 0.0)
    )
   (send-super :init)
   (setq _start_time __start_time)
   (setq _start_now __start_now)
   (setq _origin __origin)
   (setq _height (float __height))
   (setq _width (float __width))
   (setq _angle_offset (float __angle_offset))
   (setq _heading_offset (float __heading_offset))
   (setq _max_forward_speed (float __max_forward_speed))
   self)
  (:start_time
   (&rest __start_time)
   (if (keywordp (car __start_time))
       (send* _start_time __start_time)
     (progn
       (if __start_time (setq _start_time (car __start_time)))
       _start_time)))
  (:start_now
   (&optional __start_now)
   (if __start_now (setq _start_now __start_now)) _start_now)
  (:origin
   (&rest __origin)
   (if (keywordp (car __origin))
       (send* _origin __origin)
     (progn
       (if __origin (setq _origin (car __origin)))
       _origin)))
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:angle_offset
   (&optional __angle_offset)
   (if __angle_offset (setq _angle_offset __angle_offset)) _angle_offset)
  (:heading_offset
   (&optional __heading_offset)
   (if __heading_offset (setq _heading_offset __heading_offset)) _heading_offset)
  (:max_forward_speed
   (&optional __max_forward_speed)
   (if __max_forward_speed (setq _max_forward_speed __max_forward_speed)) _max_forward_speed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Time _start_time
    (send _start_time :serialization-length)
    ;; bool _start_now
    1
    ;; geometry_msgs/Point _origin
    (send _origin :serialization-length)
    ;; float64 _height
    8
    ;; float64 _width
    8
    ;; float64 _angle_offset
    8
    ;; float64 _heading_offset
    8
    ;; float64 _max_forward_speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Time _start_time
       (send _start_time :serialize s)
     ;; bool _start_now
       (if _start_now (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _origin
       (send _origin :serialize s)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _width
       (sys::poke _width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle_offset
       (sys::poke _angle_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_offset
       (sys::poke _heading_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_forward_speed
       (sys::poke _max_forward_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Time _start_time
     (send _start_time :deserialize buf ptr-) (incf ptr- (send _start_time :serialization-length))
   ;; bool _start_now
     (setq _start_now (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _origin
     (send _origin :deserialize buf ptr-) (incf ptr- (send _origin :serialization-length))
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _width
     (setq _width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle_offset
     (setq _angle_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_offset
     (setq _heading_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_forward_speed
     (setq _max_forward_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_control_msgs::InitRectTrajectoryResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::InitRectTrajectoryResponse
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

(defclass uuv_control_msgs::InitRectTrajectory
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::InitRectTrajectory :md5sum-) "bb6b6b97f153ba237ef24a0678facef1")
(setf (get uuv_control_msgs::InitRectTrajectory :datatype-) "uuv_control_msgs/InitRectTrajectory")
(setf (get uuv_control_msgs::InitRectTrajectory :request) uuv_control_msgs::InitRectTrajectoryRequest)
(setf (get uuv_control_msgs::InitRectTrajectory :response) uuv_control_msgs::InitRectTrajectoryResponse)

(defmethod uuv_control_msgs::InitRectTrajectoryRequest
  (:response () (instance uuv_control_msgs::InitRectTrajectoryResponse :init)))

(setf (get uuv_control_msgs::InitRectTrajectoryRequest :md5sum-) "bb6b6b97f153ba237ef24a0678facef1")
(setf (get uuv_control_msgs::InitRectTrajectoryRequest :datatype-) "uuv_control_msgs/InitRectTrajectoryRequest")
(setf (get uuv_control_msgs::InitRectTrajectoryRequest :definition-)
      "














std_msgs/Time start_time
bool start_now
geometry_msgs/Point origin
float64 height
float64 width
float64 angle_offset
float64 heading_offset
float64 max_forward_speed

================================================================================
MSG: std_msgs/Time
time data

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")

(setf (get uuv_control_msgs::InitRectTrajectoryResponse :md5sum-) "bb6b6b97f153ba237ef24a0678facef1")
(setf (get uuv_control_msgs::InitRectTrajectoryResponse :datatype-) "uuv_control_msgs/InitRectTrajectoryResponse")
(setf (get uuv_control_msgs::InitRectTrajectoryResponse :definition-)
      "














std_msgs/Time start_time
bool start_now
geometry_msgs/Point origin
float64 height
float64 width
float64 angle_offset
float64 heading_offset
float64 max_forward_speed

================================================================================
MSG: std_msgs/Time
time data

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")



(provide :uuv_control_msgs/InitRectTrajectory "bb6b6b97f153ba237ef24a0678facef1")


