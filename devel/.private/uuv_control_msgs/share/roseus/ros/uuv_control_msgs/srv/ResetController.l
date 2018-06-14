;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::ResetController)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'ResetController (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::RESETCONTROLLER")
  (make-package "UUV_CONTROL_MSGS::RESETCONTROLLER"))
(unless (find-package "UUV_CONTROL_MSGS::RESETCONTROLLERREQUEST")
  (make-package "UUV_CONTROL_MSGS::RESETCONTROLLERREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::RESETCONTROLLERRESPONSE")
  (make-package "UUV_CONTROL_MSGS::RESETCONTROLLERRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::ResetControllerRequest
  :super ros::object
  :slots ())

(defmethod uuv_control_msgs::ResetControllerRequest
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

(defclass uuv_control_msgs::ResetControllerResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::ResetControllerResponse
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

(defclass uuv_control_msgs::ResetController
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::ResetController :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::ResetController :datatype-) "uuv_control_msgs/ResetController")
(setf (get uuv_control_msgs::ResetController :request) uuv_control_msgs::ResetControllerRequest)
(setf (get uuv_control_msgs::ResetController :response) uuv_control_msgs::ResetControllerResponse)

(defmethod uuv_control_msgs::ResetControllerRequest
  (:response () (instance uuv_control_msgs::ResetControllerResponse :init)))

(setf (get uuv_control_msgs::ResetControllerRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::ResetControllerRequest :datatype-) "uuv_control_msgs/ResetControllerRequest")
(setf (get uuv_control_msgs::ResetControllerRequest :definition-)
      "














---
bool success

")

(setf (get uuv_control_msgs::ResetControllerResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::ResetControllerResponse :datatype-) "uuv_control_msgs/ResetControllerResponse")
(setf (get uuv_control_msgs::ResetControllerResponse :definition-)
      "














---
bool success

")



(provide :uuv_control_msgs/ResetController "358e233cde0c8a8bcfea4ce193f8fc15")


