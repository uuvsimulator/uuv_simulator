;; Auto-generated. Do not edit!


(when (boundp 'uuv_thruster_manager::SetThrusterManagerConfig)
  (if (not (find-package "UUV_THRUSTER_MANAGER"))
    (make-package "UUV_THRUSTER_MANAGER"))
  (shadow 'SetThrusterManagerConfig (find-package "UUV_THRUSTER_MANAGER")))
(unless (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIG")
  (make-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIG"))
(unless (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST")
  (make-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST"))
(unless (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGRESPONSE")
  (make-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGRESPONSE"))

(in-package "ROS")





(intern "*DEFAULT_BASE_LINK*" (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST"))
(shadow '*DEFAULT_BASE_LINK* (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST"))
(defconstant uuv_thruster_manager::SetThrusterManagerConfigRequest::*DEFAULT_BASE_LINK* "/base_link")
(intern "*DEFAULT_THRUSTER_FRAME_BASE*" (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST"))
(shadow '*DEFAULT_THRUSTER_FRAME_BASE* (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST"))
(defconstant uuv_thruster_manager::SetThrusterManagerConfigRequest::*DEFAULT_THRUSTER_FRAME_BASE* "/thruster_")
(intern "*DEFAULT_PREFIX*" (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST"))
(shadow '*DEFAULT_PREFIX* (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST"))
(defconstant uuv_thruster_manager::SetThrusterManagerConfigRequest::*DEFAULT_PREFIX* "thrusters/")
(intern "*DEFAULT_SUFFIX*" (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST"))
(shadow '*DEFAULT_SUFFIX* (find-package "UUV_THRUSTER_MANAGER::SETTHRUSTERMANAGERCONFIGREQUEST"))
(defconstant uuv_thruster_manager::SetThrusterManagerConfigRequest::*DEFAULT_SUFFIX* "/input")
(defclass uuv_thruster_manager::SetThrusterManagerConfigRequest
  :super ros::object
  :slots (_base_link _thruster_frame_base _thruster_topic_prefix _thruster_topic_suffix _timeout _max_thrust ))

(defmethod uuv_thruster_manager::SetThrusterManagerConfigRequest
  (:init
   (&key
    ((:base_link __base_link) "")
    ((:thruster_frame_base __thruster_frame_base) "")
    ((:thruster_topic_prefix __thruster_topic_prefix) "")
    ((:thruster_topic_suffix __thruster_topic_suffix) "")
    ((:timeout __timeout) 0.0)
    ((:max_thrust __max_thrust) 0.0)
    )
   (send-super :init)
   (setq _base_link (string __base_link))
   (setq _thruster_frame_base (string __thruster_frame_base))
   (setq _thruster_topic_prefix (string __thruster_topic_prefix))
   (setq _thruster_topic_suffix (string __thruster_topic_suffix))
   (setq _timeout (float __timeout))
   (setq _max_thrust (float __max_thrust))
   self)
  (:base_link
   (&optional __base_link)
   (if __base_link (setq _base_link __base_link)) _base_link)
  (:thruster_frame_base
   (&optional __thruster_frame_base)
   (if __thruster_frame_base (setq _thruster_frame_base __thruster_frame_base)) _thruster_frame_base)
  (:thruster_topic_prefix
   (&optional __thruster_topic_prefix)
   (if __thruster_topic_prefix (setq _thruster_topic_prefix __thruster_topic_prefix)) _thruster_topic_prefix)
  (:thruster_topic_suffix
   (&optional __thruster_topic_suffix)
   (if __thruster_topic_suffix (setq _thruster_topic_suffix __thruster_topic_suffix)) _thruster_topic_suffix)
  (:timeout
   (&optional __timeout)
   (if __timeout (setq _timeout __timeout)) _timeout)
  (:max_thrust
   (&optional __max_thrust)
   (if __max_thrust (setq _max_thrust __max_thrust)) _max_thrust)
  (:serialization-length
   ()
   (+
    ;; string _base_link
    4 (length _base_link)
    ;; string _thruster_frame_base
    4 (length _thruster_frame_base)
    ;; string _thruster_topic_prefix
    4 (length _thruster_topic_prefix)
    ;; string _thruster_topic_suffix
    4 (length _thruster_topic_suffix)
    ;; float64 _timeout
    8
    ;; float64 _max_thrust
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _base_link
       (write-long (length _base_link) s) (princ _base_link s)
     ;; string _thruster_frame_base
       (write-long (length _thruster_frame_base) s) (princ _thruster_frame_base s)
     ;; string _thruster_topic_prefix
       (write-long (length _thruster_topic_prefix) s) (princ _thruster_topic_prefix s)
     ;; string _thruster_topic_suffix
       (write-long (length _thruster_topic_suffix) s) (princ _thruster_topic_suffix s)
     ;; float64 _timeout
       (sys::poke _timeout (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_thrust
       (sys::poke _max_thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _base_link
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _base_link (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _thruster_frame_base
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _thruster_frame_base (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _thruster_topic_prefix
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _thruster_topic_prefix (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _thruster_topic_suffix
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _thruster_topic_suffix (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _timeout
     (setq _timeout (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_thrust
     (setq _max_thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_thruster_manager::SetThrusterManagerConfigResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_thruster_manager::SetThrusterManagerConfigResponse
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

(defclass uuv_thruster_manager::SetThrusterManagerConfig
  :super ros::object
  :slots ())

(setf (get uuv_thruster_manager::SetThrusterManagerConfig :md5sum-) "e9f260f9f8a74cbd9cca6d6d276790c0")
(setf (get uuv_thruster_manager::SetThrusterManagerConfig :datatype-) "uuv_thruster_manager/SetThrusterManagerConfig")
(setf (get uuv_thruster_manager::SetThrusterManagerConfig :request) uuv_thruster_manager::SetThrusterManagerConfigRequest)
(setf (get uuv_thruster_manager::SetThrusterManagerConfig :response) uuv_thruster_manager::SetThrusterManagerConfigResponse)

(defmethod uuv_thruster_manager::SetThrusterManagerConfigRequest
  (:response () (instance uuv_thruster_manager::SetThrusterManagerConfigResponse :init)))

(setf (get uuv_thruster_manager::SetThrusterManagerConfigRequest :md5sum-) "e9f260f9f8a74cbd9cca6d6d276790c0")
(setf (get uuv_thruster_manager::SetThrusterManagerConfigRequest :datatype-) "uuv_thruster_manager/SetThrusterManagerConfigRequest")
(setf (get uuv_thruster_manager::SetThrusterManagerConfigRequest :definition-)
      "















string DEFAULT_BASE_LINK            = /base_link
string DEFAULT_THRUSTER_FRAME_BASE  = /thruster_
string DEFAULT_PREFIX               = thrusters/
string DEFAULT_SUFFIX               = /input

string base_link
string thruster_frame_base
string thruster_topic_prefix
string thruster_topic_suffix
float64 timeout
float64 max_thrust
---
bool success

")

(setf (get uuv_thruster_manager::SetThrusterManagerConfigResponse :md5sum-) "e9f260f9f8a74cbd9cca6d6d276790c0")
(setf (get uuv_thruster_manager::SetThrusterManagerConfigResponse :datatype-) "uuv_thruster_manager/SetThrusterManagerConfigResponse")
(setf (get uuv_thruster_manager::SetThrusterManagerConfigResponse :definition-)
      "















string DEFAULT_BASE_LINK            = /base_link
string DEFAULT_THRUSTER_FRAME_BASE  = /thruster_
string DEFAULT_PREFIX               = thrusters/
string DEFAULT_SUFFIX               = /input

string base_link
string thruster_frame_base
string thruster_topic_prefix
string thruster_topic_suffix
float64 timeout
float64 max_thrust
---
bool success

")



(provide :uuv_thruster_manager/SetThrusterManagerConfig "e9f260f9f8a74cbd9cca6d6d276790c0")


