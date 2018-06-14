;; Auto-generated. Do not edit!


(when (boundp 'uuv_thruster_manager::ThrusterManagerInfo)
  (if (not (find-package "UUV_THRUSTER_MANAGER"))
    (make-package "UUV_THRUSTER_MANAGER"))
  (shadow 'ThrusterManagerInfo (find-package "UUV_THRUSTER_MANAGER")))
(unless (find-package "UUV_THRUSTER_MANAGER::THRUSTERMANAGERINFO")
  (make-package "UUV_THRUSTER_MANAGER::THRUSTERMANAGERINFO"))
(unless (find-package "UUV_THRUSTER_MANAGER::THRUSTERMANAGERINFOREQUEST")
  (make-package "UUV_THRUSTER_MANAGER::THRUSTERMANAGERINFOREQUEST"))
(unless (find-package "UUV_THRUSTER_MANAGER::THRUSTERMANAGERINFORESPONSE")
  (make-package "UUV_THRUSTER_MANAGER::THRUSTERMANAGERINFORESPONSE"))

(in-package "ROS")





(defclass uuv_thruster_manager::ThrusterManagerInfoRequest
  :super ros::object
  :slots ())

(defmethod uuv_thruster_manager::ThrusterManagerInfoRequest
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

(defclass uuv_thruster_manager::ThrusterManagerInfoResponse
  :super ros::object
  :slots (_n_thrusters _allocation_matrix _reference_frame ))

(defmethod uuv_thruster_manager::ThrusterManagerInfoResponse
  (:init
   (&key
    ((:n_thrusters __n_thrusters) 0)
    ((:allocation_matrix __allocation_matrix) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:reference_frame __reference_frame) "")
    )
   (send-super :init)
   (setq _n_thrusters (round __n_thrusters))
   (setq _allocation_matrix __allocation_matrix)
   (setq _reference_frame (string __reference_frame))
   self)
  (:n_thrusters
   (&optional __n_thrusters)
   (if __n_thrusters (setq _n_thrusters __n_thrusters)) _n_thrusters)
  (:allocation_matrix
   (&optional __allocation_matrix)
   (if __allocation_matrix (setq _allocation_matrix __allocation_matrix)) _allocation_matrix)
  (:reference_frame
   (&optional __reference_frame)
   (if __reference_frame (setq _reference_frame __reference_frame)) _reference_frame)
  (:serialization-length
   ()
   (+
    ;; int32 _n_thrusters
    4
    ;; float64[] _allocation_matrix
    (* 8    (length _allocation_matrix)) 4
    ;; string _reference_frame
    4 (length _reference_frame)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _n_thrusters
       (write-long _n_thrusters s)
     ;; float64[] _allocation_matrix
     (write-long (length _allocation_matrix) s)
     (dotimes (i (length _allocation_matrix))
       (sys::poke (elt _allocation_matrix i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string _reference_frame
       (write-long (length _reference_frame) s) (princ _reference_frame s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _n_thrusters
     (setq _n_thrusters (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _allocation_matrix
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _allocation_matrix (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _allocation_matrix i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string _reference_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reference_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uuv_thruster_manager::ThrusterManagerInfo
  :super ros::object
  :slots ())

(setf (get uuv_thruster_manager::ThrusterManagerInfo :md5sum-) "66fb8ab2f9c5649d97263c955edb636e")
(setf (get uuv_thruster_manager::ThrusterManagerInfo :datatype-) "uuv_thruster_manager/ThrusterManagerInfo")
(setf (get uuv_thruster_manager::ThrusterManagerInfo :request) uuv_thruster_manager::ThrusterManagerInfoRequest)
(setf (get uuv_thruster_manager::ThrusterManagerInfo :response) uuv_thruster_manager::ThrusterManagerInfoResponse)

(defmethod uuv_thruster_manager::ThrusterManagerInfoRequest
  (:response () (instance uuv_thruster_manager::ThrusterManagerInfoResponse :init)))

(setf (get uuv_thruster_manager::ThrusterManagerInfoRequest :md5sum-) "66fb8ab2f9c5649d97263c955edb636e")
(setf (get uuv_thruster_manager::ThrusterManagerInfoRequest :datatype-) "uuv_thruster_manager/ThrusterManagerInfoRequest")
(setf (get uuv_thruster_manager::ThrusterManagerInfoRequest :definition-)
      "














---
int32 n_thrusters
float64[] allocation_matrix
string reference_frame

")

(setf (get uuv_thruster_manager::ThrusterManagerInfoResponse :md5sum-) "66fb8ab2f9c5649d97263c955edb636e")
(setf (get uuv_thruster_manager::ThrusterManagerInfoResponse :datatype-) "uuv_thruster_manager/ThrusterManagerInfoResponse")
(setf (get uuv_thruster_manager::ThrusterManagerInfoResponse :definition-)
      "














---
int32 n_thrusters
float64[] allocation_matrix
string reference_frame

")



(provide :uuv_thruster_manager/ThrusterManagerInfo "66fb8ab2f9c5649d97263c955edb636e")


