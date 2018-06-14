;; Auto-generated. Do not edit!


(when (boundp 'uuv_manipulators_msgs::EndeffectorState)
  (if (not (find-package "UUV_MANIPULATORS_MSGS"))
    (make-package "UUV_MANIPULATORS_MSGS"))
  (shadow 'EndeffectorState (find-package "UUV_MANIPULATORS_MSGS")))
(unless (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORSTATE")
  (make-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORSTATE"))

(in-package "ROS")
;;//! \htmlinclude EndeffectorState.msg.html


(intern "*MOVING*" (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORSTATE"))
(shadow '*MOVING* (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORSTATE"))
(defconstant uuv_manipulators_msgs::EndeffectorState::*MOVING* "moving")
(intern "*DISABLED*" (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORSTATE"))
(shadow '*DISABLED* (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORSTATE"))
(defconstant uuv_manipulators_msgs::EndeffectorState::*DISABLED* "disabled")
(intern "*READY*" (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORSTATE"))
(shadow '*READY* (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORSTATE"))
(defconstant uuv_manipulators_msgs::EndeffectorState::*READY* "ready")
(defclass uuv_manipulators_msgs::EndeffectorState
  :super ros::object
  :slots (_stamp _state _position _effort ))

(defmethod uuv_manipulators_msgs::EndeffectorState
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:state __state) "")
    ((:position __position) 0.0)
    ((:effort __effort) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _state (string __state))
   (setq _position (float __position))
   (setq _effort (float __effort))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; string _state
    4 (length _state)
    ;; float64 _position
    8
    ;; float64 _effort
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _effort
       (sys::poke _effort (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _effort
     (setq _effort (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get uuv_manipulators_msgs::EndeffectorState :md5sum-) "696be13165827a929d588e22744f2679")
(setf (get uuv_manipulators_msgs::EndeffectorState :datatype-) "uuv_manipulators_msgs/EndeffectorState")
(setf (get uuv_manipulators_msgs::EndeffectorState :definition-)
      "# States of the end-effector
time      stamp
string    state
# Default states
string    MOVING      = moving
string    DISABLED    = disabled
string    READY       = ready
# Current state
float64   position
float64   effort

")



(provide :uuv_manipulators_msgs/EndeffectorState "696be13165827a929d588e22744f2679")


