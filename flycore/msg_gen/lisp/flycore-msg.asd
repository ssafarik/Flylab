
(cl:in-package :asdf)

(defsystem "flycore-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MsgFrameState" :depends-on ("_package_MsgFrameState"))
    (:file "_package_MsgFrameState" :depends-on ("_package"))
  ))