;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::InitHelicalTrajectory)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'InitHelicalTrajectory (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::INITHELICALTRAJECTORY")
  (make-package "UUV_CONTROL_MSGS::INITHELICALTRAJECTORY"))
(unless (find-package "UUV_CONTROL_MSGS::INITHELICALTRAJECTORYREQUEST")
  (make-package "UUV_CONTROL_MSGS::INITHELICALTRAJECTORYREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::INITHELICALTRAJECTORYRESPONSE")
  (make-package "UUV_CONTROL_MSGS::INITHELICALTRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass uuv_control_msgs::InitHelicalTrajectoryRequest
  :super ros::object
  :slots (_start_time _start_now _radius _center _is_clockwise _angle_offset _n_points _heading_offset _max_forward_speed _duration _n_turns _delta_z ))

(defmethod uuv_control_msgs::InitHelicalTrajectoryRequest
  (:init
   (&key
    ((:start_time __start_time) (instance std_msgs::Time :init))
    ((:start_now __start_now) nil)
    ((:radius __radius) 0.0)
    ((:center __center) (instance geometry_msgs::Point :init))
    ((:is_clockwise __is_clockwise) nil)
    ((:angle_offset __angle_offset) 0.0)
    ((:n_points __n_points) 0)
    ((:heading_offset __heading_offset) 0.0)
    ((:max_forward_speed __max_forward_speed) 0.0)
    ((:duration __duration) 0.0)
    ((:n_turns __n_turns) 0.0)
    ((:delta_z __delta_z) 0.0)
    )
   (send-super :init)
   (setq _start_time __start_time)
   (setq _start_now __start_now)
   (setq _radius (float __radius))
   (setq _center __center)
   (setq _is_clockwise __is_clockwise)
   (setq _angle_offset (float __angle_offset))
   (setq _n_points (round __n_points))
   (setq _heading_offset (float __heading_offset))
   (setq _max_forward_speed (float __max_forward_speed))
   (setq _duration (float __duration))
   (setq _n_turns (float __n_turns))
   (setq _delta_z (float __delta_z))
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
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:is_clockwise
   (&optional __is_clockwise)
   (if __is_clockwise (setq _is_clockwise __is_clockwise)) _is_clockwise)
  (:angle_offset
   (&optional __angle_offset)
   (if __angle_offset (setq _angle_offset __angle_offset)) _angle_offset)
  (:n_points
   (&optional __n_points)
   (if __n_points (setq _n_points __n_points)) _n_points)
  (:heading_offset
   (&optional __heading_offset)
   (if __heading_offset (setq _heading_offset __heading_offset)) _heading_offset)
  (:max_forward_speed
   (&optional __max_forward_speed)
   (if __max_forward_speed (setq _max_forward_speed __max_forward_speed)) _max_forward_speed)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:n_turns
   (&optional __n_turns)
   (if __n_turns (setq _n_turns __n_turns)) _n_turns)
  (:delta_z
   (&optional __delta_z)
   (if __delta_z (setq _delta_z __delta_z)) _delta_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Time _start_time
    (send _start_time :serialization-length)
    ;; bool _start_now
    1
    ;; float64 _radius
    8
    ;; geometry_msgs/Point _center
    (send _center :serialization-length)
    ;; bool _is_clockwise
    1
    ;; float64 _angle_offset
    8
    ;; int32 _n_points
    4
    ;; float64 _heading_offset
    8
    ;; float64 _max_forward_speed
    8
    ;; float64 _duration
    8
    ;; float64 _n_turns
    8
    ;; float64 _delta_z
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
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point _center
       (send _center :serialize s)
     ;; bool _is_clockwise
       (if _is_clockwise (write-byte -1 s) (write-byte 0 s))
     ;; float64 _angle_offset
       (sys::poke _angle_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _n_points
       (write-long _n_points s)
     ;; float64 _heading_offset
       (sys::poke _heading_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_forward_speed
       (sys::poke _max_forward_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _n_turns
       (sys::poke _n_turns (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_z
       (sys::poke _delta_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Time _start_time
     (send _start_time :deserialize buf ptr-) (incf ptr- (send _start_time :serialization-length))
   ;; bool _start_now
     (setq _start_now (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; bool _is_clockwise
     (setq _is_clockwise (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _angle_offset
     (setq _angle_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _n_points
     (setq _n_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _heading_offset
     (setq _heading_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_forward_speed
     (setq _max_forward_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _duration
     (setq _duration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _n_turns
     (setq _n_turns (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_z
     (setq _delta_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_control_msgs::InitHelicalTrajectoryResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::InitHelicalTrajectoryResponse
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

(defclass uuv_control_msgs::InitHelicalTrajectory
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::InitHelicalTrajectory :md5sum-) "bae09a54d9b06eeca193015644eeb493")
(setf (get uuv_control_msgs::InitHelicalTrajectory :datatype-) "uuv_control_msgs/InitHelicalTrajectory")
(setf (get uuv_control_msgs::InitHelicalTrajectory :request) uuv_control_msgs::InitHelicalTrajectoryRequest)
(setf (get uuv_control_msgs::InitHelicalTrajectory :response) uuv_control_msgs::InitHelicalTrajectoryResponse)

(defmethod uuv_control_msgs::InitHelicalTrajectoryRequest
  (:response () (instance uuv_control_msgs::InitHelicalTrajectoryResponse :init)))

(setf (get uuv_control_msgs::InitHelicalTrajectoryRequest :md5sum-) "bae09a54d9b06eeca193015644eeb493")
(setf (get uuv_control_msgs::InitHelicalTrajectoryRequest :datatype-) "uuv_control_msgs/InitHelicalTrajectoryRequest")
(setf (get uuv_control_msgs::InitHelicalTrajectoryRequest :definition-)
      "














std_msgs/Time start_time
bool start_now
float64 radius
geometry_msgs/Point center
bool is_clockwise
float64 angle_offset
int32 n_points
float64 heading_offset
float64 max_forward_speed
float64 duration
float64 n_turns
float64 delta_z

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

(setf (get uuv_control_msgs::InitHelicalTrajectoryResponse :md5sum-) "bae09a54d9b06eeca193015644eeb493")
(setf (get uuv_control_msgs::InitHelicalTrajectoryResponse :datatype-) "uuv_control_msgs/InitHelicalTrajectoryResponse")
(setf (get uuv_control_msgs::InitHelicalTrajectoryResponse :definition-)
      "














std_msgs/Time start_time
bool start_now
float64 radius
geometry_msgs/Point center
bool is_clockwise
float64 angle_offset
int32 n_points
float64 heading_offset
float64 max_forward_speed
float64 duration
float64 n_turns
float64 delta_z

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



(provide :uuv_control_msgs/InitHelicalTrajectory "bae09a54d9b06eeca193015644eeb493")


