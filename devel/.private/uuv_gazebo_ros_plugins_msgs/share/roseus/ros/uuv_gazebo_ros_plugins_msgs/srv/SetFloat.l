;; Auto-generated. Do not edit!


(when (boundp 'uuv_gazebo_ros_plugins_msgs::SetFloat)
  (if (not (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
    (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS"))
  (shadow 'SetFloat (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETFLOAT")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETFLOAT"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETFLOATREQUEST")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETFLOATREQUEST"))
(unless (find-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETFLOATRESPONSE")
  (make-package "UUV_GAZEBO_ROS_PLUGINS_MSGS::SETFLOATRESPONSE"))

(in-package "ROS")





(defclass uuv_gazebo_ros_plugins_msgs::SetFloatRequest
  :super ros::object
  :slots (_data ))

(defmethod uuv_gazebo_ros_plugins_msgs::SetFloatRequest
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64 _data
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _data
       (sys::poke _data (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _data
     (setq _data (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_gazebo_ros_plugins_msgs::SetFloatResponse
  :super ros::object
  :slots (_success _message ))

(defmethod uuv_gazebo_ros_plugins_msgs::SetFloatResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uuv_gazebo_ros_plugins_msgs::SetFloat
  :super ros::object
  :slots ())

(setf (get uuv_gazebo_ros_plugins_msgs::SetFloat :md5sum-) "3f47c51e9da05852f0d7f484f9279955")
(setf (get uuv_gazebo_ros_plugins_msgs::SetFloat :datatype-) "uuv_gazebo_ros_plugins_msgs/SetFloat")
(setf (get uuv_gazebo_ros_plugins_msgs::SetFloat :request) uuv_gazebo_ros_plugins_msgs::SetFloatRequest)
(setf (get uuv_gazebo_ros_plugins_msgs::SetFloat :response) uuv_gazebo_ros_plugins_msgs::SetFloatResponse)

(defmethod uuv_gazebo_ros_plugins_msgs::SetFloatRequest
  (:response () (instance uuv_gazebo_ros_plugins_msgs::SetFloatResponse :init)))

(setf (get uuv_gazebo_ros_plugins_msgs::SetFloatRequest :md5sum-) "3f47c51e9da05852f0d7f484f9279955")
(setf (get uuv_gazebo_ros_plugins_msgs::SetFloatRequest :datatype-) "uuv_gazebo_ros_plugins_msgs/SetFloatRequest")
(setf (get uuv_gazebo_ros_plugins_msgs::SetFloatRequest :definition-)
      "














float64 data
---
bool success
string message

")

(setf (get uuv_gazebo_ros_plugins_msgs::SetFloatResponse :md5sum-) "3f47c51e9da05852f0d7f484f9279955")
(setf (get uuv_gazebo_ros_plugins_msgs::SetFloatResponse :datatype-) "uuv_gazebo_ros_plugins_msgs/SetFloatResponse")
(setf (get uuv_gazebo_ros_plugins_msgs::SetFloatResponse :definition-)
      "














float64 data
---
bool success
string message

")



(provide :uuv_gazebo_ros_plugins_msgs/SetFloat "3f47c51e9da05852f0d7f484f9279955")


