;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::SetMBSMControllerParams)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'SetMBSMControllerParams (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::SETMBSMCONTROLLERPARAMS")
  (make-package "UUV_CONTROL_MSGS::SETMBSMCONTROLLERPARAMS"))
(unless (find-package "UUV_CONTROL_MSGS::SETMBSMCONTROLLERPARAMSREQUEST")
  (make-package "UUV_CONTROL_MSGS::SETMBSMCONTROLLERPARAMSREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::SETMBSMCONTROLLERPARAMSRESPONSE")
  (make-package "UUV_CONTROL_MSGS::SETMBSMCONTROLLERPARAMSRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::SetMBSMControllerParamsRequest
  :super ros::object
  :slots (_lambda _rho_constant _k _c _adapt_slope _rho_0 _drift_prevent ))

(defmethod uuv_control_msgs::SetMBSMControllerParamsRequest
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

(defclass uuv_control_msgs::SetMBSMControllerParamsResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::SetMBSMControllerParamsResponse
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

(defclass uuv_control_msgs::SetMBSMControllerParams
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::SetMBSMControllerParams :md5sum-) "a72093ed49272680945c74a3bdff401c")
(setf (get uuv_control_msgs::SetMBSMControllerParams :datatype-) "uuv_control_msgs/SetMBSMControllerParams")
(setf (get uuv_control_msgs::SetMBSMControllerParams :request) uuv_control_msgs::SetMBSMControllerParamsRequest)
(setf (get uuv_control_msgs::SetMBSMControllerParams :response) uuv_control_msgs::SetMBSMControllerParamsResponse)

(defmethod uuv_control_msgs::SetMBSMControllerParamsRequest
  (:response () (instance uuv_control_msgs::SetMBSMControllerParamsResponse :init)))

(setf (get uuv_control_msgs::SetMBSMControllerParamsRequest :md5sum-) "a72093ed49272680945c74a3bdff401c")
(setf (get uuv_control_msgs::SetMBSMControllerParamsRequest :datatype-) "uuv_control_msgs/SetMBSMControllerParamsRequest")
(setf (get uuv_control_msgs::SetMBSMControllerParamsRequest :definition-)
      "














float64[] lambda
float64[] rho_constant
float64[] k
float64[] c
float64[] adapt_slope
float64[] rho_0
float64 drift_prevent
---
bool success

")

(setf (get uuv_control_msgs::SetMBSMControllerParamsResponse :md5sum-) "a72093ed49272680945c74a3bdff401c")
(setf (get uuv_control_msgs::SetMBSMControllerParamsResponse :datatype-) "uuv_control_msgs/SetMBSMControllerParamsResponse")
(setf (get uuv_control_msgs::SetMBSMControllerParamsResponse :definition-)
      "














float64[] lambda
float64[] rho_constant
float64[] k
float64[] c
float64[] adapt_slope
float64[] rho_0
float64 drift_prevent
---
bool success

")



(provide :uuv_control_msgs/SetMBSMControllerParams "a72093ed49272680945c74a3bdff401c")


