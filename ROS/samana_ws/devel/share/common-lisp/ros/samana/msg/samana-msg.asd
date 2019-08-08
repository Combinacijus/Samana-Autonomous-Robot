
(cl:in-package :asdf)

(defsystem "samana-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "imu_small" :depends-on ("_package_imu_small"))
    (:file "_package_imu_small" :depends-on ("_package"))
  ))