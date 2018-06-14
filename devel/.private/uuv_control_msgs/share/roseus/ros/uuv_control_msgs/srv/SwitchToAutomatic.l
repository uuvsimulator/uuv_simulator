;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::SwitchToAutomatic)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'SwitchToAutomatic (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::SWITCHTOAUTOMATIC")
  (make-package "UUV_CONTROL_MSGS::SWITCHTOAUTOMATIC"))
(unless (find-package "UUV_CONTROL_MSGS::SWITCHTOAUTOMATICREQUEST")
  (make-package "UUV_CONTROL_MSGS::SWITCHTOAUTOMATICREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::SWITCHTOAUTOMATICRESPONSE")
  (make-package "UUV_CONTROL_MSGS::SWITCHTOAUTOMATICRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::SwitchToAutomaticRequest
  :super ros::object
  :slots ())

(defmethod uuv_control_msgs::SwitchToAutomaticRequest
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

(defclass uuv_control_msgs::SwitchToAutomaticResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::SwitchToAutomaticResponse
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

(defclass uuv_control_msgs::SwitchToAutomatic
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::SwitchToAutomatic :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::SwitchToAutomatic :datatype-) "uuv_control_msgs/SwitchToAutomatic")
(setf (get uuv_control_msgs::SwitchToAutomatic :request) uuv_control_msgs::SwitchToAutomaticRequest)
(setf (get uuv_control_msgs::SwitchToAutomatic :response) uuv_control_msgs::SwitchToAutomaticResponse)

(defmethod uuv_control_msgs::SwitchToAutomaticRequest
  (:response () (instance uuv_control_msgs::SwitchToAutomaticResponse :init)))

(setf (get uuv_control_msgs::SwitchToAutomaticRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::SwitchToAutomaticRequest :datatype-) "uuv_control_msgs/SwitchToAutomaticRequest")
(setf (get uuv_control_msgs::SwitchToAutomaticRequest :definition-)
      "














---
bool success

")

(setf (get uuv_control_msgs::SwitchToAutomaticResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::SwitchToAutomaticResponse :datatype-) "uuv_control_msgs/SwitchToAutomaticResponse")
(setf (get uuv_control_msgs::SwitchToAutomaticResponse :definition-)
      "














---
bool success

")



(provide :uuv_control_msgs/SwitchToAutomatic "358e233cde0c8a8bcfea4ce193f8fc15")


