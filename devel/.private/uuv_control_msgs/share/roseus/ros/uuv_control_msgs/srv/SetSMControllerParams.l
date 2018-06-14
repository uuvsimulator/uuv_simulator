;; Auto-generated. Do not edit!


(when (boundp 'uuv_control_msgs::SetSMControllerParams)
  (if (not (find-package "UUV_CONTROL_MSGS"))
    (make-package "UUV_CONTROL_MSGS"))
  (shadow 'SetSMControllerParams (find-package "UUV_CONTROL_MSGS")))
(unless (find-package "UUV_CONTROL_MSGS::SETSMCONTROLLERPARAMS")
  (make-package "UUV_CONTROL_MSGS::SETSMCONTROLLERPARAMS"))
(unless (find-package "UUV_CONTROL_MSGS::SETSMCONTROLLERPARAMSREQUEST")
  (make-package "UUV_CONTROL_MSGS::SETSMCONTROLLERPARAMSREQUEST"))
(unless (find-package "UUV_CONTROL_MSGS::SETSMCONTROLLERPARAMSRESPONSE")
  (make-package "UUV_CONTROL_MSGS::SETSMCONTROLLERPARAMSRESPONSE"))

(in-package "ROS")





(defclass uuv_control_msgs::SetSMControllerParamsRequest
  :super ros::object
  :slots (_K _Kd _Ki _slope ))

(defmethod uuv_control_msgs::SetSMControllerParamsRequest
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

(defclass uuv_control_msgs::SetSMControllerParamsResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_control_msgs::SetSMControllerParamsResponse
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

(defclass uuv_control_msgs::SetSMControllerParams
  :super ros::object
  :slots ())

(setf (get uuv_control_msgs::SetSMControllerParams :md5sum-) "14eb4a985748b74cd49f084da4accc8b")
(setf (get uuv_control_msgs::SetSMControllerParams :datatype-) "uuv_control_msgs/SetSMControllerParams")
(setf (get uuv_control_msgs::SetSMControllerParams :request) uuv_control_msgs::SetSMControllerParamsRequest)
(setf (get uuv_control_msgs::SetSMControllerParams :response) uuv_control_msgs::SetSMControllerParamsResponse)

(defmethod uuv_control_msgs::SetSMControllerParamsRequest
  (:response () (instance uuv_control_msgs::SetSMControllerParamsResponse :init)))

(setf (get uuv_control_msgs::SetSMControllerParamsRequest :md5sum-) "14eb4a985748b74cd49f084da4accc8b")
(setf (get uuv_control_msgs::SetSMControllerParamsRequest :datatype-) "uuv_control_msgs/SetSMControllerParamsRequest")
(setf (get uuv_control_msgs::SetSMControllerParamsRequest :definition-)
      "














float64[] K
float64[] Kd
float64[] Ki
float64[] slope
---
bool success

")

(setf (get uuv_control_msgs::SetSMControllerParamsResponse :md5sum-) "14eb4a985748b74cd49f084da4accc8b")
(setf (get uuv_control_msgs::SetSMControllerParamsResponse :datatype-) "uuv_control_msgs/SetSMControllerParamsResponse")
(setf (get uuv_control_msgs::SetSMControllerParamsResponse :definition-)
      "














float64[] K
float64[] Kd
float64[] Ki
float64[] slope
---
bool success

")



(provide :uuv_control_msgs/SetSMControllerParams "14eb4a985748b74cd49f084da4accc8b")


