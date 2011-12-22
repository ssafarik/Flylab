
(cl:in-package :asdf)

(defsystem "joystick_ps3-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "JoystickValues" :depends-on ("_package_JoystickValues"))
    (:file "_package_JoystickValues" :depends-on ("_package"))
  ))