
(cl:in-package :asdf)

(defsystem "plate_tf-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PlateStageConversion" :depends-on ("_package_PlateStageConversion"))
    (:file "_package_PlateStageConversion" :depends-on ("_package"))
    (:file "PlateCameraConversion" :depends-on ("_package_PlateCameraConversion"))
    (:file "_package_PlateCameraConversion" :depends-on ("_package"))
  ))