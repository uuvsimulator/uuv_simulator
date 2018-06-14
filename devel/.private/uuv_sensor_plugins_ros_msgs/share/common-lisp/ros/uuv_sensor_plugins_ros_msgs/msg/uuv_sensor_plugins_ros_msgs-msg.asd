
(cl:in-package :asdf)

(defsystem "uuv_sensor_plugins_ros_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ChemicalParticleConcentration" :depends-on ("_package_ChemicalParticleConcentration"))
    (:file "_package_ChemicalParticleConcentration" :depends-on ("_package"))
    (:file "DVL" :depends-on ("_package_DVL"))
    (:file "_package_DVL" :depends-on ("_package"))
    (:file "DVLBeam" :depends-on ("_package_DVLBeam"))
    (:file "_package_DVLBeam" :depends-on ("_package"))
    (:file "PositionWithCovariance" :depends-on ("_package_PositionWithCovariance"))
    (:file "_package_PositionWithCovariance" :depends-on ("_package"))
    (:file "PositionWithCovarianceStamped" :depends-on ("_package_PositionWithCovarianceStamped"))
    (:file "_package_PositionWithCovarianceStamped" :depends-on ("_package"))
    (:file "Salinity" :depends-on ("_package_Salinity"))
    (:file "_package_Salinity" :depends-on ("_package"))
  ))