
(cl:in-package :asdf)

(defsystem "flystage-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :flystage-msg
)
  :components ((:file "_package")
    (:file "SrvStageState" :depends-on ("_package_SrvStageState"))
    (:file "_package_SrvStageState" :depends-on ("_package"))
  ))