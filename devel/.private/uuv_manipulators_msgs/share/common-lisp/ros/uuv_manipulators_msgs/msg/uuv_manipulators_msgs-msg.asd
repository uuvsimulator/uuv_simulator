
(cl:in-package :asdf)

(defsystem "uuv_manipulators_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ArmConfigCommand" :depends-on ("_package_ArmConfigCommand"))
    (:file "_package_ArmConfigCommand" :depends-on ("_package"))
    (:file "EndPointState" :depends-on ("_package_EndPointState"))
    (:file "_package_EndPointState" :depends-on ("_package"))
    (:file "EndeffectorCommand" :depends-on ("_package_EndeffectorCommand"))
    (:file "_package_EndeffectorCommand" :depends-on ("_package"))
    (:file "EndeffectorState" :depends-on ("_package_EndeffectorState"))
    (:file "_package_EndeffectorState" :depends-on ("_package"))
  ))