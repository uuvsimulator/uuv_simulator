
(cl:in-package :asdf)

(defsystem "uuv_auv_control_allocator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AUVCommand" :depends-on ("_package_AUVCommand"))
    (:file "_package_AUVCommand" :depends-on ("_package"))
  ))