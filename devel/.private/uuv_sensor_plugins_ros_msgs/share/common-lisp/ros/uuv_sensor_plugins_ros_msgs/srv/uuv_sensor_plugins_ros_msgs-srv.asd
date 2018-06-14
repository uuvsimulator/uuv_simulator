
(cl:in-package :asdf)

(defsystem "uuv_sensor_plugins_ros_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ChangeSensorState" :depends-on ("_package_ChangeSensorState"))
    (:file "_package_ChangeSensorState" :depends-on ("_package"))
  ))