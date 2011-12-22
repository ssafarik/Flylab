
(cl:in-package :asdf)

(defsystem "plate_tf-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Kinematics" :depends-on ("_package_Kinematics"))
    (:file "_package_Kinematics" :depends-on ("_package"))
    (:file "FlyView" :depends-on ("_package_FlyView"))
    (:file "_package_FlyView" :depends-on ("_package"))
    (:file "InBounds" :depends-on ("_package_InBounds"))
    (:file "_package_InBounds" :depends-on ("_package"))
  ))