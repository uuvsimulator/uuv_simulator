;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::Hold)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'Hold (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::HOLD")
  (make-package "UUV_CONTROL_MSGS::HOLD"))
(unless (find-package "UUV_CONTROL_MSGS::HOLDREQUEST")
  (make-package "UUV_CONTROL_MSGS::HOLDREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::HOLDRESPONSE")
  (make-package "UUV_CONTROL_MSGS::HOLDRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::HoldRequest
  :super ros::object
  :slots ())

(defmethod uuv_control_msgs::HoldRequest
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

(defclass uuv_control_msgs::HoldResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::HoldResponse
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

(defclass uuv_control_msgs::Hold
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::Hold :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::Hold :datatype-) "uuv_control_msgs/Hold")
(setf (get uuv_control_msgs::Hold :request) uuv_control_msgs::HoldRequest)
(setf (get uuv_control_msgs::Hold :response) uuv_control_msgs::HoldResponse)

(defmethod uuv_control_msgs::HoldRequest
  (:response () (instance uuv_control_msgs::HoldResponse :init)))

(setf (get uuv_control_msgs::HoldRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::HoldRequest :datatype-) "uuv_control_msgs/HoldRequest")
(setf (get uuv_control_msgs::HoldRequest :definition-)
      "














---
bool success

")

(setf (get uuv_control_msgs::HoldResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::HoldResponse :datatype-) "uuv_control_msgs/HoldResponse")
(setf (get uuv_control_msgs::HoldResponse :definition-)
      "














---
bool success

")



(provide :uuv_control_msgs/Hold "358e233cde0c8a8bcfea4ce193f8fc15")


