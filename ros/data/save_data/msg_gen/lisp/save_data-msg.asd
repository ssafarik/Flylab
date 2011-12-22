
(cl:in-package :asdf)

(defsystem "save_data-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BagInfo" :depends-on ("_package_BagInfo"))
    (:file "_package_BagInfo" :depends-on ("_package"))
    (:file "VideoInfo" :depends-on ("_package_VideoInfo"))
    (:file "_package_VideoInfo" :depends-on ("_package"))
    (:file "SaveSettings" :depends-on ("_package_SaveSettings"))
    (:file "_package_SaveSettings" :depends-on ("_package"))
    (:file "CommandSavedata" :depends-on ("_package_CommandSavedata"))
    (:file "_package_CommandSavedata" :depends-on ("_package"))
  ))