; Auto-generated. Do not edit!


(cl:in-package joystick_ps3-msg)


;//! \htmlinclude JoystickValues.msg.html

(cl:defclass <JoystickValues> (roslisp-msg-protocol:ros-message)
  ((x_left
    :reader x_left
    :initarg :x_left
    :type cl:float
    :initform 0.0)
   (y_left
    :reader y_left
    :initarg :y_left
    :type cl:float
    :initform 0.0)
   (x_right
    :reader x_right
    :initarg :x_right
    :type cl:float
    :initform 0.0)
   (y_right
    :reader y_right
    :initarg :y_right
    :type cl:float
    :initform 0.0)
   (up
    :reader up
    :initarg :up
    :type cl:boolean
    :initform cl:nil)
   (down
    :reader down
    :initarg :down
    :type cl:boolean
    :initform cl:nil)
   (left
    :reader left
    :initarg :left
    :type cl:boolean
    :initform cl:nil)
   (right
    :reader right
    :initarg :right
    :type cl:boolean
    :initform cl:nil)
   (triangle
    :reader triangle
    :initarg :triangle
    :type cl:boolean
    :initform cl:nil)
   (x
    :reader x
    :initarg :x
    :type cl:boolean
    :initform cl:nil)
   (square
    :reader square
    :initarg :square
    :type cl:boolean
    :initform cl:nil)
   (circle
    :reader circle
    :initarg :circle
    :type cl:boolean
    :initform cl:nil)
   (select
    :reader select
    :initarg :select
    :type cl:boolean
    :initform cl:nil)
   (start
    :reader start
    :initarg :start
    :type cl:boolean
    :initform cl:nil)
   (playstation
    :reader playstation
    :initarg :playstation
    :type cl:boolean
    :initform cl:nil)
   (L1
    :reader L1
    :initarg :L1
    :type cl:boolean
    :initform cl:nil)
   (L2
    :reader L2
    :initarg :L2
    :type cl:boolean
    :initform cl:nil)
   (R1
    :reader R1
    :initarg :R1
    :type cl:boolean
    :initform cl:nil)
   (R2
    :reader R2
    :initarg :R2
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass JoystickValues (<JoystickValues>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoystickValues>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoystickValues)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name joystick_ps3-msg:<JoystickValues> is deprecated: use joystick_ps3-msg:JoystickValues instead.")))

(cl:ensure-generic-function 'x_left-val :lambda-list '(m))
(cl:defmethod x_left-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:x_left-val is deprecated.  Use joystick_ps3-msg:x_left instead.")
  (x_left m))

(cl:ensure-generic-function 'y_left-val :lambda-list '(m))
(cl:defmethod y_left-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:y_left-val is deprecated.  Use joystick_ps3-msg:y_left instead.")
  (y_left m))

(cl:ensure-generic-function 'x_right-val :lambda-list '(m))
(cl:defmethod x_right-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:x_right-val is deprecated.  Use joystick_ps3-msg:x_right instead.")
  (x_right m))

(cl:ensure-generic-function 'y_right-val :lambda-list '(m))
(cl:defmethod y_right-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:y_right-val is deprecated.  Use joystick_ps3-msg:y_right instead.")
  (y_right m))

(cl:ensure-generic-function 'up-val :lambda-list '(m))
(cl:defmethod up-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:up-val is deprecated.  Use joystick_ps3-msg:up instead.")
  (up m))

(cl:ensure-generic-function 'down-val :lambda-list '(m))
(cl:defmethod down-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:down-val is deprecated.  Use joystick_ps3-msg:down instead.")
  (down m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:left-val is deprecated.  Use joystick_ps3-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:right-val is deprecated.  Use joystick_ps3-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'triangle-val :lambda-list '(m))
(cl:defmethod triangle-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:triangle-val is deprecated.  Use joystick_ps3-msg:triangle instead.")
  (triangle m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:x-val is deprecated.  Use joystick_ps3-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'square-val :lambda-list '(m))
(cl:defmethod square-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:square-val is deprecated.  Use joystick_ps3-msg:square instead.")
  (square m))

(cl:ensure-generic-function 'circle-val :lambda-list '(m))
(cl:defmethod circle-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:circle-val is deprecated.  Use joystick_ps3-msg:circle instead.")
  (circle m))

(cl:ensure-generic-function 'select-val :lambda-list '(m))
(cl:defmethod select-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:select-val is deprecated.  Use joystick_ps3-msg:select instead.")
  (select m))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:start-val is deprecated.  Use joystick_ps3-msg:start instead.")
  (start m))

(cl:ensure-generic-function 'playstation-val :lambda-list '(m))
(cl:defmethod playstation-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:playstation-val is deprecated.  Use joystick_ps3-msg:playstation instead.")
  (playstation m))

(cl:ensure-generic-function 'L1-val :lambda-list '(m))
(cl:defmethod L1-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:L1-val is deprecated.  Use joystick_ps3-msg:L1 instead.")
  (L1 m))

(cl:ensure-generic-function 'L2-val :lambda-list '(m))
(cl:defmethod L2-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:L2-val is deprecated.  Use joystick_ps3-msg:L2 instead.")
  (L2 m))

(cl:ensure-generic-function 'R1-val :lambda-list '(m))
(cl:defmethod R1-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:R1-val is deprecated.  Use joystick_ps3-msg:R1 instead.")
  (R1 m))

(cl:ensure-generic-function 'R2-val :lambda-list '(m))
(cl:defmethod R2-val ((m <JoystickValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick_ps3-msg:R2-val is deprecated.  Use joystick_ps3-msg:R2 instead.")
  (R2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoystickValues>) ostream)
  "Serializes a message object of type '<JoystickValues>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'up) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'down) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'triangle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'x) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'square) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'circle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'select) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'playstation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'L1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'L2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R2) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoystickValues>) istream)
  "Deserializes a message object of type '<JoystickValues>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_right) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_right) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'up) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'down) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'triangle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'x) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'square) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'circle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'select) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'start) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'playstation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'L1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'L2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R2) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoystickValues>)))
  "Returns string type for a message object of type '<JoystickValues>"
  "joystick_ps3/JoystickValues")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoystickValues)))
  "Returns string type for a message object of type 'JoystickValues"
  "joystick_ps3/JoystickValues")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoystickValues>)))
  "Returns md5sum for a message object of type '<JoystickValues>"
  "f1af9c9ec7eff6b8ec4f182194e49cb3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoystickValues)))
  "Returns md5sum for a message object of type 'JoystickValues"
  "f1af9c9ec7eff6b8ec4f182194e49cb3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoystickValues>)))
  "Returns full string definition for message of type '<JoystickValues>"
  (cl:format cl:nil "float64 x_left~%float64 y_left~%float64 x_right~%float64 y_right~%bool up~%bool down~%bool left~%bool right~%bool triangle~%bool x~%bool square~%bool circle~%bool select~%bool start~%bool playstation~%bool L1~%bool L2~%bool R1~%bool R2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoystickValues)))
  "Returns full string definition for message of type 'JoystickValues"
  (cl:format cl:nil "float64 x_left~%float64 y_left~%float64 x_right~%float64 y_right~%bool up~%bool down~%bool left~%bool right~%bool triangle~%bool x~%bool square~%bool circle~%bool select~%bool start~%bool playstation~%bool L1~%bool L2~%bool R1~%bool R2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoystickValues>))
  (cl:+ 0
     8
     8
     8
     8
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoystickValues>))
  "Converts a ROS message object to a list"
  (cl:list 'JoystickValues
    (cl:cons ':x_left (x_left msg))
    (cl:cons ':y_left (y_left msg))
    (cl:cons ':x_right (x_right msg))
    (cl:cons ':y_right (y_right msg))
    (cl:cons ':up (up msg))
    (cl:cons ':down (down msg))
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
    (cl:cons ':triangle (triangle msg))
    (cl:cons ':x (x msg))
    (cl:cons ':square (square msg))
    (cl:cons ':circle (circle msg))
    (cl:cons ':select (select msg))
    (cl:cons ':start (start msg))
    (cl:cons ':playstation (playstation msg))
    (cl:cons ':L1 (L1 msg))
    (cl:cons ':L2 (L2 msg))
    (cl:cons ':R1 (R1 msg))
    (cl:cons ':R2 (R2 msg))
))
