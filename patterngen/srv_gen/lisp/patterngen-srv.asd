
(cl:in-package :asdf)

(defsystem "patterngen-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SrvSignal" :depends-on ("_package_SrvSignal"))
    (:file "_package_SrvSignal" :depends-on ("_package"))
  ))