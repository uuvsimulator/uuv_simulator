
(cl:in-package :asdf)

(defsystem "uuv_gazebo_ros_plugins_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FloatStamped" :depends-on ("_package_FloatStamped"))
    (:file "_package_FloatStamped" :depends-on ("_package"))
    (:file "UnderwaterObjectModel" :depends-on ("_package_UnderwaterObjectModel"))
    (:file "_package_UnderwaterObjectModel" :depends-on ("_package"))
  ))