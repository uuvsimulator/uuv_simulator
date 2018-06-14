;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::GetSMControllerParams)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'GetSMControllerParams (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::GETSMCONTROLLERPARAMS")
  (make-package "UUV_CONTROL_MSGS::GETSMCONTROLLERPARAMS"))
(unless (find-package "UUV_CONTROL_MSGS::GETSMCONTROLLERPARAMSREQUEST")
  (make-package "UUV_CONTROL_MSGS::GETSMCONTROLLERPARAMSREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::GETSMCONTROLLERPARAMSRESPONSE")
  (make-package "UUV_CONTROL_MSGS::GETSMCONTROLLERPARAMSRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::GetSMControllerParamsRequest
  :super ros::object
  :slots ())

(defmethod uuv_control_msgs::GetSMControllerParamsRequest
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

(defclass uuv_control_msgs::GetSMControllerParamsResponse
  :super ros::object
  :slots (_K _Kd _Ki _slope ))

(defmethod uuv_control_msgs::GetSMControllerParamsResponse
  (:init
   (&key
    ((:K __K) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:Kd __Kd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:Ki __Ki) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:slope __slope) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _K __K)
   (setq _Kd __Kd)
   (setq _Ki __Ki)
   (setq _slope __slope)
   self)
  (:K
   (&optional __K)
   (if __K (setq _K __K)) _K)
  (:Kd
   (&optional __Kd)
   (if __Kd (setq _Kd __Kd)) _Kd)
  (:Ki
   (&optional __Ki)
   (if __Ki (setq _Ki __Ki)) _Ki)
  (:slope
   (&optional __slope)
   (if __slope (setq _slope __slope)) _slope)
  (:serialization-length
   ()
   (+
    ;; float64[] _K
    (* 8    (length _K)) 4
    ;; float64[] _Kd
    (* 8    (length _Kd)) 4
    ;; float64[] _Ki
    (* 8    (length _Ki)) 4
    ;; float64[] _slope
    (* 8    (length _slope)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _K
     (write-long (length _K) s)
     (dotimes (i (length _K))
       (sys::poke (elt _K i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
     ;; float64[] _slope
     (write-long (length _slope) s)
     (dotimes (i (length _slope))
       (sys::poke (elt _slope i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _K
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _K (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _K i) (sys::peek buf ptr- :double)) (incf ptr- 8)
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
   ;; float64[] _slope
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _slope (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _slope i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass uuv_control_msgs::GetSMControllerParams
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::GetSMControllerParams :md5sum-) "dc294c7929e39ce02ce0ce70a116b3b3")
(setf (get uuv_control_msgs::GetSMControllerParams :datatype-) "uuv_control_msgs/GetSMControllerParams")
(setf (get uuv_control_msgs::GetSMControllerParams :request) uuv_control_msgs::GetSMControllerParamsRequest)
(setf (get uuv_control_msgs::GetSMControllerParams :response) uuv_control_msgs::GetSMControllerParamsResponse)

(defmethod uuv_control_msgs::GetSMControllerParamsRequest
  (:response () (instance uuv_control_msgs::GetSMControllerParamsResponse :init)))

(setf (get uuv_control_msgs::GetSMControllerParamsRequest :md5sum-) "dc294c7929e39ce02ce0ce70a116b3b3")
(setf (get uuv_control_msgs::GetSMControllerParamsRequest :datatype-) "uuv_control_msgs/GetSMControllerParamsRequest")
(setf (get uuv_control_msgs::GetSMControllerParamsRequest :definition-)
      "














---
float64[] K
float64[] Kd
float64[] Ki
float64[] slope

")

(setf (get uuv_control_msgs::GetSMControllerParamsResponse :md5sum-) "dc294c7929e39ce02ce0ce70a116b3b3")
(setf (get uuv_control_msgs::GetSMControllerParamsResponse :datatype-) "uuv_control_msgs/GetSMControllerParamsResponse")
(setf (get uuv_control_msgs::GetSMControllerParamsResponse :definition-)
      "














---
float64[] K
float64[] Kd
float64[] Ki
float64[] slope

")



(provide :uuv_control_msgs/GetSMControllerParams "dc294c7929e39ce02ce0ce70a116b3b3")


