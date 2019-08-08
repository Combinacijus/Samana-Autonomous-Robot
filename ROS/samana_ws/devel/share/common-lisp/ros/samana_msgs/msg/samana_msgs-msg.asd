
(cl:in-package :asdf)

(defsystem "samana_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ImuCalib" :depends-on ("_package_ImuCalib"))
    (:file "_package_ImuCalib" :depends-on ("_package"))
    (:file "ImuSmall" :depends-on ("_package_ImuSmall"))
    (:file "_package_ImuSmall" :depends-on ("_package"))
    (:file "Sonar" :depends-on ("_package_Sonar"))
    (:file "_package_Sonar" :depends-on ("_package"))
    (:file "Vector3_32" :depends-on ("_package_Vector3_32"))
    (:file "_package_Vector3_32" :depends-on ("_package"))
    (:file "Vector4_32" :depends-on ("_package_Vector4_32"))
    (:file "_package_Vector4_32" :depends-on ("_package"))
  ))