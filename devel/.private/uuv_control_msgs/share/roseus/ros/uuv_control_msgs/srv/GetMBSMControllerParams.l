;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::GetMBSMControllerParams)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'GetMBSMControllerParams (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::GETMBSMCONTROLLERPARAMS")
  (make-package "UUV_CONTROL_MSGS::GETMBSMCONTROLLERPARAMS"))
(unless (find-package "UUV_CONTROL_MSGS::GETMBSMCONTROLLERPARAMSREQUEST")
  (make-package "UUV_CONTROL_MSGS::GETMBSMCONTROLLERPARAMSREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::GETMBSMCONTROLLERPARAMSRESPONSE")
  (make-package "UUV_CONTROL_MSGS::GETMBSMCONTROLLERPARAMSRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::GetMBSMControllerParamsRequest
  :super ros::object
  :slots ())

(defmethod uuv_control_msgs::GetMBSMControllerParamsRequest
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

(defclass uuv_control_msgs::GetMBSMControllerParamsResponse
  :super ros::object
  :slots (_lambda _rho_constant _k _c _adapt_slope _rho_0 _drift_prevent ))

(defmethod uuv_control_msgs::GetMBSMControllerParamsResponse
  (:init
   (&key
    ((:lambda __lambda) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:rho_constant __rho_constant) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:k __k) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:c __c) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:adapt_slope __adapt_slope) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:rho_0 __rho_0) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:drift_prevent __drift_prevent) 0.0)
    )
   (send-super :init)
   (setq _lambda __lambda)
   (setq _rho_constant __rho_constant)
   (setq _k __k)
   (setq _c __c)
   (setq _adapt_slope __adapt_slope)
   (setq _rho_0 __rho_0)
   (setq _drift_prevent (float __drift_prevent))
   self)
  (:lambda
   (&optional __lambda)
   (if __lambda (setq _lambda __lambda)) _lambda)
  (:rho_constant
   (&optional __rho_constant)
   (if __rho_constant (setq _rho_constant __rho_constant)) _rho_constant)
  (:k
   (&optional __k)
   (if __k (setq _k __k)) _k)
  (:c
   (&optional __c)
   (if __c (setq _c __c)) _c)
  (:adapt_slope
   (&optional __adapt_slope)
   (if __adapt_slope (setq _adapt_slope __adapt_slope)) _adapt_slope)
  (:rho_0
   (&optional __rho_0)
   (if __rho_0 (setq _rho_0 __rho_0)) _rho_0)
  (:drift_prevent
   (&optional __drift_prevent)
   (if __drift_prevent (setq _drift_prevent __drift_prevent)) _drift_prevent)
  (:serialization-length
   ()
   (+
    ;; float64[] _lambda
    (* 8    (length _lambda)) 4
    ;; float64[] _rho_constant
    (* 8    (length _rho_constant)) 4
    ;; float64[] _k
    (* 8    (length _k)) 4
    ;; float64[] _c
    (* 8    (length _c)) 4
    ;; float64[] _adapt_slope
    (* 8    (length _adapt_slope)) 4
    ;; float64[] _rho_0
    (* 8    (length _rho_0)) 4
    ;; float64 _drift_prevent
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _lambda
     (write-long (length _lambda) s)
     (dotimes (i (length _lambda))
       (sys::poke (elt _lambda i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _rho_constant
     (write-long (length _rho_constant) s)
     (dotimes (i (length _rho_constant))
       (sys::poke (elt _rho_constant i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _k
     (write-long (length _k) s)
     (dotimes (i (length _k))
       (sys::poke (elt _k i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _c
     (write-long (length _c) s)
     (dotimes (i (length _c))
       (sys::poke (elt _c i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _adapt_slope
     (write-long (length _adapt_slope) s)
     (dotimes (i (length _adapt_slope))
       (sys::poke (elt _adapt_slope i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _rho_0
     (write-long (length _rho_0) s)
     (dotimes (i (length _rho_0))
       (sys::poke (elt _rho_0 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _drift_prevent
       (sys::poke _drift_prevent (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _lambda
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lambda (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _lambda i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _rho_constant
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rho_constant (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _rho_constant i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _k
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _k (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _k i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _c
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _c (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _c i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _adapt_slope
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _adapt_slope (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _adapt_slope i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _rho_0
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rho_0 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _rho_0 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _drift_prevent
     (setq _drift_prevent (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_control_msgs::GetMBSMControllerParams
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::GetMBSMControllerParams :md5sum-) "7293aecc8487ffe3e998814d65aa6940")
(setf (get uuv_control_msgs::GetMBSMControllerParams :datatype-) "uuv_control_msgs/GetMBSMControllerParams")
(setf (get uuv_control_msgs::GetMBSMControllerParams :request) uuv_control_msgs::GetMBSMControllerParamsRequest)
(setf (get uuv_control_msgs::GetMBSMControllerParams :response) uuv_control_msgs::GetMBSMControllerParamsResponse)

(defmethod uuv_control_msgs::GetMBSMControllerParamsRequest
  (:response () (instance uuv_control_msgs::GetMBSMControllerParamsResponse :init)))

(setf (get uuv_control_msgs::GetMBSMControllerParamsRequest :md5sum-) "7293aecc8487ffe3e998814d65aa6940")
(setf (get uuv_control_msgs::GetMBSMControllerParamsRequest :datatype-) "uuv_control_msgs/GetMBSMControllerParamsRequest")
(setf (get uuv_control_msgs::GetMBSMControllerParamsRequest :definition-)
      "














---
float64[] lambda
float64[] rho_constant
float64[] k
float64[] c
float64[] adapt_slope
float64[] rho_0
float64 drift_prevent

")

(setf (get uuv_control_msgs::GetMBSMControllerParamsResponse :md5sum-) "7293aecc8487ffe3e998814d65aa6940")
(setf (get uuv_control_msgs::GetMBSMControllerParamsResponse :datatype-) "uuv_control_msgs/GetMBSMControllerParamsResponse")
(setf (get uuv_control_msgs::GetMBSMControllerParamsResponse :definition-)
      "














---
float64[] lambda
float64[] rho_constant
float64[] k
float64[] c
float64[] adapt_slope
float64[] rho_0
float64 drift_prevent

")



(provide :uuv_control_msgs/GetMBSMControllerParams "7293aecc8487ffe3e998814d65aa6940")


