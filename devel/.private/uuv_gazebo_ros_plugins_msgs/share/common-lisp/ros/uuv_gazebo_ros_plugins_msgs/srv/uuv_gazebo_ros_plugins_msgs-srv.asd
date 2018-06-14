
(cl:in-package :asdf)

(defsystem "uuv_gazebo_ros_plugins_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :uuv_gazebo_ros_plugins_msgs-msg
)
  :components ((:file "_package")
    (:file "GetFloat" :depends-on ("_package_GetFloat"))
    (:file "_package_GetFloat" :depends-on ("_package"))
    (:file "GetModelProperties" :depends-on ("_package_GetModelProperties"))
    (:file "_package_GetModelProperties" :depends-on ("_package"))
    (:file "GetThrusterEfficiency" :depends-on ("_package_GetThrusterEfficiency"))
    (:file "_package_GetThrusterEfficiency" :depends-on ("_package"))
    (:file "GetThrusterState" :depends-on ("_package_GetThrusterState"))
    (:file "_package_GetThrusterState" :depends-on ("_package"))
    (:file "SetFloat" :depends-on ("_package_SetFloat"))
    (:file "_package_SetFloat" :depends-on ("_package"))
    (:file "SetThrusterEfficiency" :depends-on ("_package_SetThrusterEfficiency"))
    (:file "_package_SetThrusterEfficiency" :depends-on ("_package"))
    (:file "SetThrusterState" :depends-on ("_package_SetThrusterState"))
    (:file "_package_SetThrusterState" :depends-on ("_package"))
    (:file "SetUseGlobalCurrentVel" :depends-on ("_package_SetUseGlobalCurrentVel"))
    (:file "_package_SetUseGlobalCurrentVel" :depends-on ("_package"))
  ))