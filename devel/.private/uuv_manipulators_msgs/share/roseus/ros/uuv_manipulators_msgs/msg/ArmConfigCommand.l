;; Auto-generated. Do not edit!


(when (boundp 'uuv_manipulators_msgs::ArmConfigCommand)
  (if (not (find-package "UUV_MANIPULATORS_MSGS"))
    (make-package "UUV_MANIPULATORS_MSGS"))
  (shadow 'ArmConfigCommand (find-package "UUV_MANIPULATORS_MSGS")))
(unless (find-package "UUV_MANIPULATORS_MSGS::ARMCONFIGCOMMAND")
  (make-package "UUV_MANIPULATORS_MSGS::ARMCONFIGCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ArmConfigCommand.msg.html


(intern "*HOME*" (find-package "UUV_MANIPULATORS_MSGS::ARMCONFIGCOMMAND"))
(shadow '*HOME* (find-package "UUV_MANIPULATORS_MSGS::ARMCONFIGCOMMAND"))
(defconstant uuv_manipulators_msgs::ArmConfigCommand::*HOME* "home")
(defclass uuv_manipulators_msgs::ArmConfigCommand
  :super ros::object
  :slots (_command _args ))

(defmethod uuv_manipulators_msgs::ArmConfigCommand
  (:init
   (&key
    ((:command __command) "")
    ((:args __args) "")
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _args (string __args))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:args
   (&optional __args)
   (if __args (setq _args __args)) _args)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ;; string _args
    4 (length _args)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; string _args
       (write-long (length _args) s) (princ _args s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _args
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _args (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get uuv_manipulators_msgs::ArmConfigCommand :md5sum-) "428ecc7602e5c382dfc52081cc34f5a6")
(setf (get uuv_manipulators_msgs::ArmConfigCommand :datatype-) "uuv_manipulators_msgs/ArmConfigCommand")
(setf (get uuv_manipulators_msgs::ArmConfigCommand :definition-)
      "# Commands to drive the arm to default configuration
string  command     # Operation tag
# Default commands
string  HOME      = home
# Place for arguments, if needed
string  args

")



(provide :uuv_manipulators_msgs/ArmConfigCommand "428ecc7602e5c382dfc52081cc34f5a6")


