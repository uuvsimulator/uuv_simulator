;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::ClearWaypoints)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'ClearWaypoints (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::CLEARWAYPOINTS")
  (make-package "UUV_CONTROL_MSGS::CLEARWAYPOINTS"))
(unless (find-package "UUV_CONTROL_MSGS::CLEARWAYPOINTSREQUEST")
  (make-package "UUV_CONTROL_MSGS::CLEARWAYPOINTSREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::CLEARWAYPOINTSRESPONSE")
  (make-package "UUV_CONTROL_MSGS::CLEARWAYPOINTSRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::ClearWaypointsRequest
  :super ros::object
  :slots ())

(defmethod uuv_control_msgs::ClearWaypointsRequest
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

(defclass uuv_control_msgs::ClearWaypointsResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::ClearWaypointsResponse
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

(defclass uuv_control_msgs::ClearWaypoints
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::ClearWaypoints :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::ClearWaypoints :datatype-) "uuv_control_msgs/ClearWaypoints")
(setf (get uuv_control_msgs::ClearWaypoints :request) uuv_control_msgs::ClearWaypointsRequest)
(setf (get uuv_control_msgs::ClearWaypoints :response) uuv_control_msgs::ClearWaypointsResponse)

(defmethod uuv_control_msgs::ClearWaypointsRequest
  (:response () (instance uuv_control_msgs::ClearWaypointsResponse :init)))

(setf (get uuv_control_msgs::ClearWaypointsRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::ClearWaypointsRequest :datatype-) "uuv_control_msgs/ClearWaypointsRequest")
(setf (get uuv_control_msgs::ClearWaypointsRequest :definition-)
      "














---
bool success

")

(setf (get uuv_control_msgs::ClearWaypointsResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_control_msgs::ClearWaypointsResponse :datatype-) "uuv_control_msgs/ClearWaypointsResponse")
(setf (get uuv_control_msgs::ClearWaypointsResponse :definition-)
      "














---
bool success

")



(provide :uuv_control_msgs/ClearWaypoints "358e233cde0c8a8bcfea4ce193f8fc15")


