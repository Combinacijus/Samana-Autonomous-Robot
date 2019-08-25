
(cl:in-package :asdf)

(defsystem "samana_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Bump" :depends-on ("_package_Bump"))
    (:file "_package_Bump" :depends-on ("_package"))
    (:file "ImuCalib" :depends-on ("_package_ImuCalib"))
    (:file "_package_ImuCalib" :depends-on ("_package"))
    (:file "ImuSmall" :depends-on ("_package_ImuSmall"))
    (:file "_package_ImuSmall" :depends-on ("_package"))
    (:file "Int16Array" :depends-on ("_package_Int16Array"))
    (:file "_package_Int16Array" :depends-on ("_package"))
    (:file "Teleop" :depends-on ("_package_Teleop"))
    (:file "_package_Teleop" :depends-on ("_package"))
    (:file "Vector3_32" :depends-on ("_package_Vector3_32"))
    (:file "_package_Vector3_32" :depends-on ("_package"))
    (:file "Vector4_32" :depends-on ("_package_Vector4_32"))
    (:file "_package_Vector4_32" :depends-on ("_package"))
  ))