
(cl:in-package :asdf)

(defsystem "uuv_thruster_manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetThrusterCurve" :depends-on ("_package_GetThrusterCurve"))
    (:file "_package_GetThrusterCurve" :depends-on ("_package"))
    (:file "GetThrusterManagerConfig" :depends-on ("_package_GetThrusterManagerConfig"))
    (:file "_package_GetThrusterManagerConfig" :depends-on ("_package"))
    (:file "SetThrusterManagerConfig" :depends-on ("_package_SetThrusterManagerConfig"))
    (:file "_package_SetThrusterManagerConfig" :depends-on ("_package"))
    (:file "ThrusterManagerInfo" :depends-on ("_package_ThrusterManagerInfo"))
    (:file "_package_ThrusterManagerInfo" :depends-on ("_package"))
  ))