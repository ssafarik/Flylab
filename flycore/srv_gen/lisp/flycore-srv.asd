
(cl:in-package :asdf)

(defsystem "flycore-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :flycore-msg
)
  :components ((:file "_package")
    (:file "SrvFrameState" :depends-on ("_package_SrvFrameState"))
    (:file "_package_SrvFrameState" :depends-on ("_package"))
  ))