
(cl:in-package :asdf)

(defsystem "uuv_control_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Trajectory" :depends-on ("_package_Trajectory"))
    (:file "_package_Trajectory" :depends-on ("_package"))
    (:file "TrajectoryPoint" :depends-on ("_package_TrajectoryPoint"))
    (:file "_package_TrajectoryPoint" :depends-on ("_package"))
    (:file "Waypoint" :depends-on ("_package_Waypoint"))
    (:file "_package_Waypoint" :depends-on ("_package"))
    (:file "WaypointSet" :depends-on ("_package_WaypointSet"))
    (:file "_package_WaypointSet" :depends-on ("_package"))
  ))