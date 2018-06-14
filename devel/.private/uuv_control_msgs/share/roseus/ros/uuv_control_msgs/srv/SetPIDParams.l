;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::SetPIDParams)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'SetPIDParams (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::SETPIDPARAMS")
  (make-package "UUV_CONTROL_MSGS::SETPIDPARAMS"))
(unless (find-package "UUV_CONTROL_MSGS::SETPIDPARAMSREQUEST")
  (make-package "UUV_CONTROL_MSGS::SETPIDPARAMSREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::SETPIDPARAMSRESPONSE")
  (make-package "UUV_CONTROL_MSGS::SETPIDPARAMSRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::SetPIDParamsRequest
  :super ros::object
  :slots (_Kp _Kd _Ki ))

(defmethod uuv_control_msgs::SetPIDParamsRequest
  (:init
   (&key
    ((:Kp __Kp) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:Kd __Kd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:Ki __Ki) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _Kp __Kp)
   (setq _Kd __Kd)
   (setq _Ki __Ki)
   self)
  (:Kp
   (&optional __Kp)
   (if __Kp (setq _Kp __Kp)) _Kp)
  (:Kd
   (&optional __Kd)
   (if __Kd (setq _Kd __Kd)) _Kd)
  (:Ki
   (&optional __Ki)
   (if __Ki (setq _Ki __Ki)) _Ki)
  (:serialization-length
   ()
   (+
    ;; float64[] _Kp
    (* 8    (length _Kp)) 4
    ;; float64[] _Kd
    (* 8    (length _Kd)) 4
    ;; float64[] _Ki
    (* 8    (length _Ki)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _Kp
     (write-long (length _Kp) s)
     (dotimes (i (length _Kp))
       (sys::poke (elt _Kp i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _Kd
     (write-long (length _Kd) s)
     (dotimes (i (length _Kd))
       (sys::poke (elt _Kd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _Ki
     (write-long (length _Ki) s)
     (dotimes (i (length _Ki))
       (sys::poke (elt _Ki i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _Kp
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Kp (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _Kp i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _Kd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Kd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _Kd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _Ki
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Ki (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _Ki i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass uuv_control_msgs::SetPIDParamsResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::SetPIDParamsResponse
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

(defclass uuv_control_msgs::SetPIDParams
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::SetPIDParams :md5sum-) "147ee245a68427e8ed98870bce36b399")
(setf (get uuv_control_msgs::SetPIDParams :datatype-) "uuv_control_msgs/SetPIDParams")
(setf (get uuv_control_msgs::SetPIDParams :request) uuv_control_msgs::SetPIDParamsRequest)
(setf (get uuv_control_msgs::SetPIDParams :response) uuv_control_msgs::SetPIDParamsResponse)

(defmethod uuv_control_msgs::SetPIDParamsRequest
  (:response () (instance uuv_control_msgs::SetPIDParamsResponse :init)))

(setf (get uuv_control_msgs::SetPIDParamsRequest :md5sum-) "147ee245a68427e8ed98870bce36b399")
(setf (get uuv_control_msgs::SetPIDParamsRequest :datatype-) "uuv_control_msgs/SetPIDParamsRequest")
(setf (get uuv_control_msgs::SetPIDParamsRequest :definition-)
      "














float64[] Kp
float64[] Kd
float64[] Ki
---
bool success

")

(setf (get uuv_control_msgs::SetPIDParamsResponse :md5sum-) "147ee245a68427e8ed98870bce36b399")
(setf (get uuv_control_msgs::SetPIDParamsResponse :datatype-) "uuv_control_msgs/SetPIDParamsResponse")
(setf (get uuv_control_msgs::SetPIDParamsResponse :definition-)
      "














float64[] Kp
float64[] Kd
float64[] Ki
---
bool success

")



(provide :uuv_control_msgs/SetPIDParams "147ee245a68427e8ed98870bce36b399")


