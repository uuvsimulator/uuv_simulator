
(cl:in-package :asdf)

(defsystem "uuv_world_ros_plugins_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "GetCurrentModel" :depends-on ("_package_GetCurrentModel"))
    (:file "_package_GetCurrentModel" :depends-on ("_package"))
    (:file "GetOriginSphericalCoord" :depends-on ("_package_GetOriginSphericalCoord"))
    (:file "_package_GetOriginSphericalCoord" :depends-on ("_package"))
    (:file "SetCurrentDirection" :depends-on ("_package_SetCurrentDirection"))
    (:file "_package_SetCurrentDirection" :depends-on ("_package"))
    (:file "SetCurrentModel" :depends-on ("_package_SetCurrentModel"))
    (:file "_package_SetCurrentModel" :depends-on ("_package"))
    (:file "SetCurrentVelocity" :depends-on ("_package_SetCurrentVelocity"))
    (:file "_package_SetCurrentVelocity" :depends-on ("_package"))
    (:file "SetOriginSphericalCoord" :depends-on ("_package_SetOriginSphericalCoord"))
    (:file "_package_SetOriginSphericalCoord" :depends-on ("_package"))
    (:file "TransformFromSphericalCoord" :depends-on ("_package_TransformFromSphericalCoord"))
    (:file "_package_TransformFromSphericalCoord" :depends-on ("_package"))
    (:file "TransformToSphericalCoord" :depends-on ("_package_TransformToSphericalCoord"))
    (:file "_package_TransformToSphericalCoord" :depends-on ("_package"))
  ))