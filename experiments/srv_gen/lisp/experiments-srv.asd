
(cl:in-package :asdf)

(defsystem "experiments-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :experiments-msg
)
  :components ((:file "_package")
    (:file "Trigger" :depends-on ("_package_Trigger"))
    (:file "_package_Trigger" :depends-on ("_package"))
    (:file "ExperimentParams" :depends-on ("_package_ExperimentParams"))
    (:file "_package_ExperimentParams" :depends-on ("_package"))
  ))