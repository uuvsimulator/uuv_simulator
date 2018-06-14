;; Auto-generated. Do not edit!


(when (boundp 'uuv_manipulators_msgs::EndeffectorCommand)
  (if (not (find-package "UUV_MANIPULATORS_MSGS"))
    (make-package "UUV_MANIPULATORS_MSGS"))
  (shadow 'EndeffectorCommand (find-package "UUV_MANIPULATORS_MSGS")))
(unless (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND")
  (make-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude EndeffectorCommand.msg.html


(intern "*EE_MOVE*" (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND"))
(shadow '*EE_MOVE* (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND"))
(defconstant uuv_manipulators_msgs::EndeffectorCommand::*EE_MOVE* "move")
(intern "*EE_STOP*" (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND"))
(shadow '*EE_STOP* (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND"))
(defconstant uuv_manipulators_msgs::EndeffectorCommand::*EE_STOP* "stop")
(intern "*EE_CLOSED*" (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND"))
(shadow '*EE_CLOSED* (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND"))
(defconstant uuv_manipulators_msgs::EndeffectorCommand::*EE_CLOSED* 0.0)
(intern "*EE_OPEN*" (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND"))
(shadow '*EE_OPEN* (find-package "UUV_MANIPULATORS_MSGS::ENDEFFECTORCOMMAND"))
(defconstant uuv_manipulators_msgs::EndeffectorCommand::*EE_OPEN* 100.0)
(defclass uuv_manipulators_msgs::EndeffectorCommand
  :super ros::object
  :slots (_command _ratio ))

(defmethod uuv_manipulators_msgs::EndeffectorCommand
  (:init
   (&key
    ((:command __command) "")
    ((:ratio __ratio) 0.0)
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _ratio (float __ratio))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:ratio
   (&optional __ratio)
   (if __ratio (setq _ratio __ratio)) _ratio)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ;; float64 _ratio
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; float64 _ratio
       (sys::poke _ratio (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _ratio
     (setq _ratio (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get uuv_manipulators_msgs::EndeffectorCommand :md5sum-) "2949dea8a0d479d93952df57c48d98d6")
(setf (get uuv_manipulators_msgs::EndeffectorCommand :datatype-) "uuv_manipulators_msgs/EndeffectorCommand")
(setf (get uuv_manipulators_msgs::EndeffectorCommand :definition-)
      "# Commands to the end-effector
string  command     # Operation tag
# Default commands
string  EE_MOVE   = move
string  EE_STOP   = stop
# Place for arguments, if needed
float64 ratio
# Default ratios of aperture
float64 EE_CLOSED = 0.0
float64 EE_OPEN   = 100.0

")



(provide :uuv_manipulators_msgs/EndeffectorCommand "2949dea8a0d479d93952df57c48d98d6")


