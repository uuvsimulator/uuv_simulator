
(cl:in-package :asdf)

(defsystem "uuv_manipulators_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "SolveIK" :depends-on ("_package_SolveIK"))
    (:file "_package_SolveIK" :depends-on ("_package"))
  ))