;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::GetPIDParams)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'GetPIDParams (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::GETPIDPARAMS")
  (make-package "UUV_CONTROL_MSGS::GETPIDPARAMS"))
(unless (find-package "UUV_CONTROL_MSGS::GETPIDPARAMSREQUEST")
  (make-package "UUV_CONTROL_MSGS::GETPIDPARAMSREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::GETPIDPARAMSRESPONSE")
  (make-package "UUV_CONTROL_MSGS::GETPIDPARAMSRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::GetPIDParamsRequest
  :super ros::object
  :slots ())

(defmethod uuv_control_msgs::GetPIDParamsRequest
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

(defclass uuv_control_msgs::GetPIDParamsResponse
  :super ros::object
  :slots (_Kp _Kd _Ki ))

(defmethod uuv_control_msgs::GetPIDParamsResponse
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

(defclass uuv_control_msgs::GetPIDParams
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::GetPIDParams :md5sum-) "1dae001799e4bc231c788fb194cf733a")
(setf (get uuv_control_msgs::GetPIDParams :datatype-) "uuv_control_msgs/GetPIDParams")
(setf (get uuv_control_msgs::GetPIDParams :request) uuv_control_msgs::GetPIDParamsRequest)
(setf (get uuv_control_msgs::GetPIDParams :response) uuv_control_msgs::GetPIDParamsResponse)

(defmethod uuv_control_msgs::GetPIDParamsRequest
  (:response () (instance uuv_control_msgs::GetPIDParamsResponse :init)))

(setf (get uuv_control_msgs::GetPIDParamsRequest :md5sum-) "1dae001799e4bc231c788fb194cf733a")
(setf (get uuv_control_msgs::GetPIDParamsRequest :datatype-) "uuv_control_msgs/GetPIDParamsRequest")
(setf (get uuv_control_msgs::GetPIDParamsRequest :definition-)
      "














---
float64[] Kp
float64[] Kd
float64[] Ki

")

(setf (get uuv_control_msgs::GetPIDParamsResponse :md5sum-) "1dae001799e4bc231c788fb194cf733a")
(setf (get uuv_control_msgs::GetPIDParamsResponse :datatype-) "uuv_control_msgs/GetPIDParamsResponse")
(setf (get uuv_control_msgs::GetPIDParamsResponse :definition-)
      "














---
float64[] Kp
float64[] Kd
float64[] Ki

")



(provide :uuv_control_msgs/GetPIDParams "1dae001799e4bc231c788fb194cf733a")


