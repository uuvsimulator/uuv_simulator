;; Auto-generated. Do not edit!


(when (boundp 'uuv_thruster_manager::GetThrusterCurve)
  (if (not (find-package "UUV_THRUSTER_MANAGER"))
    (make-package "UUV_THRUSTER_MANAGER"))
  (shadow 'GetThrusterCurve (find-package "UUV_THRUSTER_MANAGER")))
(unless (find-package "UUV_THRUSTER_MANAGER::GETTHRUSTERCURVE")
  (make-package "UUV_THRUSTER_MANAGER::GETTHRUSTERCURVE"))
(unless (find-package "UUV_THRUSTER_MANAGER::GETTHRUSTERCURVEREQUEST")
  (make-package "UUV_THRUSTER_MANAGER::GETTHRUSTERCURVEREQUEST"))
(unless (find-package "UUV_THRUSTER_MANAGER::GETTHRUSTERCURVERESPONSE")
  (make-package "UUV_THRUSTER_MANAGER::GETTHRUSTERCURVERESPONSE"))

(in-package "ROS")





(defclass uuv_thruster_manager::GetThrusterCurveRequest
  :super ros::object
  :slots (_min _max _n_points ))

(defmethod uuv_thruster_manager::GetThrusterCurveRequest
  (:init
   (&key
    ((:min __min) 0.0)
    ((:max __max) 0.0)
    ((:n_points __n_points) 0)
    )
   (send-super :init)
   (setq _min (float __min))
   (setq _max (float __max))
   (setq _n_points (round __n_points))
   self)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:max
   (&optional __max)
   (if __max (setq _max __max)) _max)
  (:n_points
   (&optional __n_points)
   (if __n_points (setq _n_points __n_points)) _n_points)
  (:serialization-length
   ()
   (+
    ;; float64 _min
    8
    ;; float64 _max
    8
    ;; int32 _n_points
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _min
       (sys::poke _min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max
       (sys::poke _max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _n_points
       (write-long _n_points s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _min
     (setq _min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max
     (setq _max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _n_points
     (setq _n_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass uuv_thruster_manager::GetThrusterCurveResponse
  :super ros::object
  :slots (_input _thrust ))

(defmethod uuv_thruster_manager::GetThrusterCurveResponse
  (:init
   (&key
    ((:input __input) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:thrust __thrust) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _input __input)
   (setq _thrust __thrust)
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:serialization-length
   ()
   (+
    ;; float64[] _input
    (* 8    (length _input)) 4
    ;; float64[] _thrust
    (* 8    (length _thrust)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _input
     (write-long (length _input) s)
     (dotimes (i (length _input))
       (sys::poke (elt _input i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _thrust
     (write-long (length _thrust) s)
     (dotimes (i (length _thrust))
       (sys::poke (elt _thrust i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _input
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _input (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _input i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _thrust
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _thrust (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _thrust i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass uuv_thruster_manager::GetThrusterCurve
  :super ros::object
  :slots ())

(setf (get uuv_thruster_manager::GetThrusterCurve :md5sum-) "93e0d8a78977b3619bdc09f290ef57fa")
(setf (get uuv_thruster_manager::GetThrusterCurve :datatype-) "uuv_thruster_manager/GetThrusterCurve")
(setf (get uuv_thruster_manager::GetThrusterCurve :request) uuv_thruster_manager::GetThrusterCurveRequest)
(setf (get uuv_thruster_manager::GetThrusterCurve :response) uuv_thruster_manager::GetThrusterCurveResponse)

(defmethod uuv_thruster_manager::GetThrusterCurveRequest
  (:response () (instance uuv_thruster_manager::GetThrusterCurveResponse :init)))

(setf (get uuv_thruster_manager::GetThrusterCurveRequest :md5sum-) "93e0d8a78977b3619bdc09f290ef57fa")
(setf (get uuv_thruster_manager::GetThrusterCurveRequest :datatype-) "uuv_thruster_manager/GetThrusterCurveRequest")
(setf (get uuv_thruster_manager::GetThrusterCurveRequest :definition-)
      "














float64 min
float64 max
int32 n_points
---
float64[] input
float64[] thrust

")

(setf (get uuv_thruster_manager::GetThrusterCurveResponse :md5sum-) "93e0d8a78977b3619bdc09f290ef57fa")
(setf (get uuv_thruster_manager::GetThrusterCurveResponse :datatype-) "uuv_thruster_manager/GetThrusterCurveResponse")
(setf (get uuv_thruster_manager::GetThrusterCurveResponse :definition-)
      "














float64 min
float64 max
int32 n_points
---
float64[] input
float64[] thrust

")



(provide :uuv_thruster_manager/GetThrusterCurve "93e0d8a78977b3619bdc09f290ef57fa")


