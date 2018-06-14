;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_plugins_ros_msgs::ChangeSensorState)
  (if (not (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
    (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS"))
  (shadow 'ChangeSensorState (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS")))
(unless (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::CHANGESENSORSTATE")
  (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS::CHANGESENSORSTATE"))
(unless (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::CHANGESENSORSTATEREQUEST")
  (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS::CHANGESENSORSTATEREQUEST"))
(unless (find-package "UUV_SENSOR_PLUGINS_ROS_MSGS::CHANGESENSORSTATERESPONSE")
  (make-package "UUV_SENSOR_PLUGINS_ROS_MSGS::CHANGESENSORSTATERESPONSE"))

(in-package "ROS")





(defclass uuv_sensor_plugins_ros_msgs::ChangeSensorStateRequest
  :super ros::object
  :slots (_on ))

(defmethod uuv_sensor_plugins_ros_msgs::ChangeSensorStateRequest
  (:init
   (&key
    ((:on __on) nil)
    )
   (send-super :init)
   (setq _on __on)
   self)
  (:on
   (&optional __on)
   (if __on (setq _on __on)) _on)
  (:serialization-length
   ()
   (+
    ;; bool _on
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _on
       (if _on (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _on
     (setq _on (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uuv_sensor_plugins_ros_msgs::ChangeSensorStateResponse
  :super ros::object
  :slots (_success _message ))

(defmethod uuv_sensor_plugins_ros_msgs::ChangeSensorStateResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uuv_sensor_plugins_ros_msgs::ChangeSensorState
  :super ros::object
  :slots ())

(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorState :md5sum-) "d1c85d57f0ffdd55759c88fc35773fc9")
(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorState :datatype-) "uuv_sensor_plugins_ros_msgs/ChangeSensorState")
(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorState :request) uuv_sensor_plugins_ros_msgs::ChangeSensorStateRequest)
(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorState :response) uuv_sensor_plugins_ros_msgs::ChangeSensorStateResponse)

(defmethod uuv_sensor_plugins_ros_msgs::ChangeSensorStateRequest
  (:response () (instance uuv_sensor_plugins_ros_msgs::ChangeSensorStateResponse :init)))

(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorStateRequest :md5sum-) "d1c85d57f0ffdd55759c88fc35773fc9")
(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorStateRequest :datatype-) "uuv_sensor_plugins_ros_msgs/ChangeSensorStateRequest")
(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorStateRequest :definition-)
      "bool on
---
bool success
string message

")

(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorStateResponse :md5sum-) "d1c85d57f0ffdd55759c88fc35773fc9")
(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorStateResponse :datatype-) "uuv_sensor_plugins_ros_msgs/ChangeSensorStateResponse")
(setf (get uuv_sensor_plugins_ros_msgs::ChangeSensorStateResponse :definition-)
      "bool on
---
bool success
string message

")



(provide :uuv_sensor_plugins_ros_msgs/ChangeSensorState "d1c85d57f0ffdd55759c88fc35773fc9")


