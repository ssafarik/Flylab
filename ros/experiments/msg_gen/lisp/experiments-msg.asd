
(cl:in-package :asdf)

(defsystem "experiments-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TriggerSettings" :depends-on ("_package_TriggerSettings"))
    (:file "_package_TriggerSettings" :depends-on ("_package"))
    (:file "SaveSettings" :depends-on ("_package_SaveSettings"))
    (:file "_package_SaveSettings" :depends-on ("_package"))
    (:file "MoveSettings" :depends-on ("_package_MoveSettings"))
    (:file "_package_MoveSettings" :depends-on ("_package"))
    (:file "HomeSettings" :depends-on ("_package_HomeSettings"))
    (:file "_package_HomeSettings" :depends-on ("_package"))
    (:file "ExperimentSettings" :depends-on ("_package_ExperimentSettings"))
    (:file "_package_ExperimentSettings" :depends-on ("_package"))
  ))