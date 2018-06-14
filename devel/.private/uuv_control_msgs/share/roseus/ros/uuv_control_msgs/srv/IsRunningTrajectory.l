;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::IsRunningTrajectory)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'IsRunningTrajectory (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::ISRUNNINGTRAJECTORY")
  (make-package "UUV_CONTROL_MSGS::ISRUNNINGTRAJECTORY"))
(unless (find-package "UUV_CONTROL_MSGS::ISRUNNINGTRAJECTORYREQUEST")
  (make-package "UUV_CONTROL_MSGS::ISRUNNINGTRAJECTORYREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::ISRUNNINGTRAJECTORYRESPONSE")
  (make-package "UUV_CONTROL_MSGS::ISRUNNINGTRAJECTORYRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::IsRunningTrajectoryRequest
  :super ros::object
  :slots ())

(defmethod uuv_control_msgs::IsRunningTrajectoryRequest
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

(defclass uuv_control_msgs::IsRunningTrajectoryResponse
  :super ros::object
  :slots (_progress ))

(defmethod uuv_control_msgs::IsRunningTrajectoryResponse
  (:init
   (&key
    ((:progress __progress) 0.0)
    )
   (send-super :init)
   (setq _progress (float __progress))
   self)
  (:progress
   (&optional __progress)
   (if __progress (setq _progress __progress)) _progress)
  (:serialization-length
   ()
   (+
    ;; float64 _progress
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _progress
       (sys::poke _progress (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _progress
     (setq _progress (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_control_msgs::IsRunningTrajectory
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::IsRunningTrajectory :md5sum-) "7afca0099e0cddc25243b1e3569895fe")
(setf (get uuv_control_msgs::IsRunningTrajectory :datatype-) "uuv_control_msgs/IsRunningTrajectory")
(setf (get uuv_control_msgs::IsRunningTrajectory :request) uuv_control_msgs::IsRunningTrajectoryRequest)
(setf (get uuv_control_msgs::IsRunningTrajectory :response) uuv_control_msgs::IsRunningTrajectoryResponse)

(defmethod uuv_control_msgs::IsRunningTrajectoryRequest
  (:response () (instance uuv_control_msgs::IsRunningTrajectoryResponse :init)))

(setf (get uuv_control_msgs::IsRunningTrajectoryRequest :md5sum-) "7afca0099e0cddc25243b1e3569895fe")
(setf (get uuv_control_msgs::IsRunningTrajectoryRequest :datatype-) "uuv_control_msgs/IsRunningTrajectoryRequest")
(setf (get uuv_control_msgs::IsRunningTrajectoryRequest :definition-)
      "














---
float64 progress

")

(setf (get uuv_control_msgs::IsRunningTrajectoryResponse :md5sum-) "7afca0099e0cddc25243b1e3569895fe")
(setf (get uuv_control_msgs::IsRunningTrajectoryResponse :datatype-) "uuv_control_msgs/IsRunningTrajectoryResponse")
(setf (get uuv_control_msgs::IsRunningTrajectoryResponse :definition-)
      "














---
float64 progress

")



(provide :uuv_control_msgs/IsRunningTrajectory "7afca0099e0cddc25243b1e3569895fe")


