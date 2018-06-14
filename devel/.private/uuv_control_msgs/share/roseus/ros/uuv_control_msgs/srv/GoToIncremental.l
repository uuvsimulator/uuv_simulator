;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::GoToIncremental)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'GoToIncremental (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTAL")
  (make-package "UUV_CONTROL_MSGS::GOTOINCREMENTAL"))
(unless (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST")
  (make-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALRESPONSE")
  (make-package "UUV_CONTROL_MSGS::GOTOINCREMENTALRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(intern "*LIPB*" (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST"))
(shadow '*LIPB* (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST"))
(defconstant uuv_control_msgs::GoToIncrementalRequest::*LIPB* "'lipb'")
(intern "*CUBIC*" (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST"))
(shadow '*CUBIC* (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST"))
(defconstant uuv_control_msgs::GoToIncrementalRequest::*CUBIC* "'cubic'")
(intern "*DUBINS*" (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST"))
(shadow '*DUBINS* (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST"))
(defconstant uuv_control_msgs::GoToIncrementalRequest::*DUBINS* "'dubins'")
(intern "*LINEAR*" (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST"))
(shadow '*LINEAR* (find-package "UUV_CONTROL_MSGS::GOTOINCREMENTALREQUEST"))
(defconstant uuv_control_msgs::GoToIncrementalRequest::*LINEAR* "'linear'")
(defclass uuv_control_msgs::GoToIncrementalRequest
  :super ros::object
  :slots (_step _max_forward_speed _interpolator ))

(defmethod uuv_control_msgs::GoToIncrementalRequest
  (:init
   (&key
    ((:step __step) (instance geometry_msgs::Point :init))
    ((:max_forward_speed __max_forward_speed) 0.0)
    ((:interpolator __interpolator) "")
    )
   (send-super :init)
   (setq _step __step)
   (setq _max_forward_speed (float __max_forward_speed))
   (setq _interpolator (string __interpolator))
   self)
  (:step
   (&rest __step)
   (if (keywordp (car __step))
       (send* _step __step)
     (progn
       (if __step (setq _step (car __step)))
       _step)))
  (:max_forward_speed
   (&optional __max_forward_speed)
   (if __max_forward_speed (setq _max_forward_speed __max_forward_speed)) _max_forward_speed)
  (:interpolator
   (&optional __interpolator)
   (if __interpolator (setq _interpolator __interpolator)) _interpolator)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _step
    (send _step :serialization-length)
    ;; float64 _max_forward_speed
    8
    ;; string _interpolator
    4 (length _interpolator)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _step
       (send _step :serialize s)
     ;; float64 _max_forward_speed
       (sys::poke _max_forward_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _interpolator
       (write-long (length _interpolator) s) (princ _interpolator s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _step
     (send _step :deserialize buf ptr-) (incf ptr- (send _step :serialization-length))
   ;; float64 _max_forward_speed
     (setq _max_forward_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _interpolator
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _interpolator (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uuv_control_msgs::GoToIncrementalResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::GoToIncrementalResponse
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

(defclass uuv_control_msgs::GoToIncremental
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::GoToIncremental :md5sum-) "ea062c779dd21cbac8fefabcd9b5f18e")
(setf (get uuv_control_msgs::GoToIncremental :datatype-) "uuv_control_msgs/GoToIncremental")
(setf (get uuv_control_msgs::GoToIncremental :request) uuv_control_msgs::GoToIncrementalRequest)
(setf (get uuv_control_msgs::GoToIncremental :response) uuv_control_msgs::GoToIncrementalResponse)

(defmethod uuv_control_msgs::GoToIncrementalRequest
  (:response () (instance uuv_control_msgs::GoToIncrementalResponse :init)))

(setf (get uuv_control_msgs::GoToIncrementalRequest :md5sum-) "ea062c779dd21cbac8fefabcd9b5f18e")
(setf (get uuv_control_msgs::GoToIncrementalRequest :datatype-) "uuv_control_msgs/GoToIncrementalRequest")
(setf (get uuv_control_msgs::GoToIncrementalRequest :definition-)
      "














string LIPB='lipb'
string CUBIC='cubic'
string DUBINS='dubins'
string LINEAR='linear'

geometry_msgs/Point step
float64 max_forward_speed
string interpolator

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")

(setf (get uuv_control_msgs::GoToIncrementalResponse :md5sum-) "ea062c779dd21cbac8fefabcd9b5f18e")
(setf (get uuv_control_msgs::GoToIncrementalResponse :datatype-) "uuv_control_msgs/GoToIncrementalResponse")
(setf (get uuv_control_msgs::GoToIncrementalResponse :definition-)
      "














string LIPB='lipb'
string CUBIC='cubic'
string DUBINS='dubins'
string LINEAR='linear'

geometry_msgs/Point step
float64 max_forward_speed
string interpolator

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")



(provide :uuv_control_msgs/GoToIncremental "ea062c779dd21cbac8fefabcd9b5f18e")


