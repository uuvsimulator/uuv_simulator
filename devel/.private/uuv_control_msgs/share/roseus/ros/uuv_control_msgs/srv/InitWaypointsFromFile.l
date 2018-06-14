;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::InitWaypointsFromFile)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'InitWaypointsFromFile (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILE")
  (make-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILE"))
(unless (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST")
  (make-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILERESPONSE")
  (make-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILERESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(intern "*LIPB*" (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST"))
(shadow '*LIPB* (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST"))
(defconstant uuv_control_msgs::InitWaypointsFromFileRequest::*LIPB* "'lipb'")
(intern "*CUBIC*" (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST"))
(shadow '*CUBIC* (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST"))
(defconstant uuv_control_msgs::InitWaypointsFromFileRequest::*CUBIC* "'cubic'")
(intern "*DUBINS*" (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST"))
(shadow '*DUBINS* (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST"))
(defconstant uuv_control_msgs::InitWaypointsFromFileRequest::*DUBINS* "'dubins'")
(intern "*LINEAR*" (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST"))
(shadow '*LINEAR* (find-package "UUV_CONTROL_MSGS::INITWAYPOINTSFROMFILEREQUEST"))
(defconstant uuv_control_msgs::InitWaypointsFromFileRequest::*LINEAR* "'linear'")
(defclass uuv_control_msgs::InitWaypointsFromFileRequest
  :super ros::object
  :slots (_start_time _start_now _filename _interpolator ))

(defmethod uuv_control_msgs::InitWaypointsFromFileRequest
  (:init
   (&key
    ((:start_time __start_time) (instance std_msgs::Time :init))
    ((:start_now __start_now) nil)
    ((:filename __filename) (instance std_msgs::String :init))
    ((:interpolator __interpolator) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _start_time __start_time)
   (setq _start_now __start_now)
   (setq _filename __filename)
   (setq _interpolator __interpolator)
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
  (:filename
   (&rest __filename)
   (if (keywordp (car __filename))
       (send* _filename __filename)
     (progn
       (if __filename (setq _filename (car __filename)))
       _filename)))
  (:interpolator
   (&rest __interpolator)
   (if (keywordp (car __interpolator))
       (send* _interpolator __interpolator)
     (progn
       (if __interpolator (setq _interpolator (car __interpolator)))
       _interpolator)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Time _start_time
    (send _start_time :serialization-length)
    ;; bool _start_now
    1
    ;; std_msgs/String _filename
    (send _filename :serialization-length)
    ;; std_msgs/String _interpolator
    (send _interpolator :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Time _start_time
       (send _start_time :serialize s)
     ;; bool _start_now
       (if _start_now (write-byte -1 s) (write-byte 0 s))
     ;; std_msgs/String _filename
       (send _filename :serialize s)
     ;; std_msgs/String _interpolator
       (send _interpolator :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Time _start_time
     (send _start_time :deserialize buf ptr-) (incf ptr- (send _start_time :serialization-length))
   ;; bool _start_now
     (setq _start_now (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; std_msgs/String _filename
     (send _filename :deserialize buf ptr-) (incf ptr- (send _filename :serialization-length))
   ;; std_msgs/String _interpolator
     (send _interpolator :deserialize buf ptr-) (incf ptr- (send _interpolator :serialization-length))
   ;;
   self)
  )

(defclass uuv_control_msgs::InitWaypointsFromFileResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::InitWaypointsFromFileResponse
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

(defclass uuv_control_msgs::InitWaypointsFromFile
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::InitWaypointsFromFile :md5sum-) "a0198a3f7b74ab5dc081e88db6d85c58")
(setf (get uuv_control_msgs::InitWaypointsFromFile :datatype-) "uuv_control_msgs/InitWaypointsFromFile")
(setf (get uuv_control_msgs::InitWaypointsFromFile :request) uuv_control_msgs::InitWaypointsFromFileRequest)
(setf (get uuv_control_msgs::InitWaypointsFromFile :response) uuv_control_msgs::InitWaypointsFromFileResponse)

(defmethod uuv_control_msgs::InitWaypointsFromFileRequest
  (:response () (instance uuv_control_msgs::InitWaypointsFromFileResponse :init)))

(setf (get uuv_control_msgs::InitWaypointsFromFileRequest :md5sum-) "a0198a3f7b74ab5dc081e88db6d85c58")
(setf (get uuv_control_msgs::InitWaypointsFromFileRequest :datatype-) "uuv_control_msgs/InitWaypointsFromFileRequest")
(setf (get uuv_control_msgs::InitWaypointsFromFileRequest :definition-)
      "














string LIPB='lipb'
string CUBIC='cubic'
string DUBINS='dubins'
string LINEAR='linear'

std_msgs/Time start_time
bool start_now
std_msgs/String filename
std_msgs/String interpolator

================================================================================
MSG: std_msgs/Time
time data

================================================================================
MSG: std_msgs/String
string data
---
bool success

")

(setf (get uuv_control_msgs::InitWaypointsFromFileResponse :md5sum-) "a0198a3f7b74ab5dc081e88db6d85c58")
(setf (get uuv_control_msgs::InitWaypointsFromFileResponse :datatype-) "uuv_control_msgs/InitWaypointsFromFileResponse")
(setf (get uuv_control_msgs::InitWaypointsFromFileResponse :definition-)
      "














string LIPB='lipb'
string CUBIC='cubic'
string DUBINS='dubins'
string LINEAR='linear'

std_msgs/Time start_time
bool start_now
std_msgs/String filename
std_msgs/String interpolator

================================================================================
MSG: std_msgs/Time
time data

================================================================================
MSG: std_msgs/String
string data
---
bool success

")



(provide :uuv_control_msgs/InitWaypointsFromFile "a0198a3f7b74ab5dc081e88db6d85c58")


