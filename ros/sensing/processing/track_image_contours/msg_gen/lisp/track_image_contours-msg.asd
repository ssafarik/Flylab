
(cl:in-package :asdf)

(defsystem "track_image_contours-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :flycore-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ContourInfo" :depends-on ("_package_ContourInfo"))
    (:file "_package_ContourInfo" :depends-on ("_package"))
    (:file "Contour" :depends-on ("_package_Contour"))
    (:file "_package_Contour" :depends-on ("_package"))
    (:file "ArenaState" :depends-on ("_package_ArenaState"))
    (:file "_package_ArenaState" :depends-on ("_package"))
  ))