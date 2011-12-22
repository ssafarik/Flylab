; Auto-generated. Do not edit!


(cl:in-package experiments-msg)


;//! \htmlinclude MoveSettings.msg.html

(cl:defclass <MoveSettings> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil)
   (tracking
    :reader tracking
    :initarg :tracking
    :type cl:boolean
    :initform cl:nil)
   (frameidOriginPosition
    :reader frameidOriginPosition
    :initarg :frameidOriginPosition
    :type cl:string
    :initform "")
   (frameidOriginAngle
    :reader frameidOriginAngle
    :initarg :frameidOriginAngle
    :type cl:string
    :initform "")
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (angleType
    :reader angleType
    :initarg :angleType
    :type cl:string
    :initform "")
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (velocityType
    :reader velocityType
    :initarg :velocityType
    :type cl:string
    :initform "")
   (tolerance
    :reader tolerance
    :initarg :tolerance
    :type cl:float
    :initform 0.0)
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:float
    :initform 0.0))
)

(cl:defclass MoveSettings (<MoveSettings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveSettings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveSettings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name experiments-msg:<MoveSettings> is deprecated: use experiments-msg:MoveSettings instead.")))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:enabled-val is deprecated.  Use experiments-msg:enabled instead.")
  (enabled m))

(cl:ensure-generic-function 'tracking-val :lambda-list '(m))
(cl:defmethod tracking-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:tracking-val is deprecated.  Use experiments-msg:tracking instead.")
  (tracking m))

(cl:ensure-generic-function 'frameidOriginPosition-val :lambda-list '(m))
(cl:defmethod frameidOriginPosition-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:frameidOriginPosition-val is deprecated.  Use experiments-msg:frameidOriginPosition instead.")
  (frameidOriginPosition m))

(cl:ensure-generic-function 'frameidOriginAngle-val :lambda-list '(m))
(cl:defmethod frameidOriginAngle-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:frameidOriginAngle-val is deprecated.  Use experiments-msg:frameidOriginAngle instead.")
  (frameidOriginAngle m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:distance-val is deprecated.  Use experiments-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:angle-val is deprecated.  Use experiments-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'angleType-val :lambda-list '(m))
(cl:defmethod angleType-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:angleType-val is deprecated.  Use experiments-msg:angleType instead.")
  (angleType m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:velocity-val is deprecated.  Use experiments-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'velocityType-val :lambda-list '(m))
(cl:defmethod velocityType-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:velocityType-val is deprecated.  Use experiments-msg:velocityType instead.")
  (velocityType m))

(cl:ensure-generic-function 'tolerance-val :lambda-list '(m))
(cl:defmethod tolerance-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:tolerance-val is deprecated.  Use experiments-msg:tolerance instead.")
  (tolerance m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:timeout-val is deprecated.  Use experiments-msg:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveSettings>) ostream)
  "Serializes a message object of type '<MoveSettings>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tracking) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frameidOriginPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frameidOriginPosition))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frameidOriginAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frameidOriginAngle))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'angleType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'angleType))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'velocityType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'velocityType))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tolerance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timeout))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveSettings>) istream)
  "Deserializes a message object of type '<MoveSettings>"
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tracking) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frameidOriginPosition) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frameidOriginPosition) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frameidOriginAngle) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frameidOriginAngle) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angleType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'angleType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocityType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'velocityType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeout) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveSettings>)))
  "Returns string type for a message object of type '<MoveSettings>"
  "experiments/MoveSettings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveSettings)))
  "Returns string type for a message object of type 'MoveSettings"
  "experiments/MoveSettings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveSettings>)))
  "Returns md5sum for a message object of type '<MoveSettings>"
  "8a3c810f3c1f2cb5fb317a984814d92e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveSettings)))
  "Returns md5sum for a message object of type 'MoveSettings"
  "8a3c810f3c1f2cb5fb317a984814d92e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveSettings>)))
  "Returns full string definition for message of type '<MoveSettings>"
  (cl:format cl:nil "bool enabled~%bool tracking~%string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'~%string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'~%float64 distance~%float64 angle~%string angleType # 'random' or 'constant'~%float64 velocity~%string velocityType # 'random' or 'constant'~%float64 tolerance~%float64 timeout~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveSettings)))
  "Returns full string definition for message of type 'MoveSettings"
  (cl:format cl:nil "bool enabled~%bool tracking~%string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'~%string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'~%float64 distance~%float64 angle~%string angleType # 'random' or 'constant'~%float64 velocity~%string velocityType # 'random' or 'constant'~%float64 tolerance~%float64 timeout~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveSettings>))
  (cl:+ 0
     1
     1
     4 (cl:length (cl:slot-value msg 'frameidOriginPosition))
     4 (cl:length (cl:slot-value msg 'frameidOriginAngle))
     8
     8
     4 (cl:length (cl:slot-value msg 'angleType))
     8
     4 (cl:length (cl:slot-value msg 'velocityType))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveSettings>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveSettings
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':tracking (tracking msg))
    (cl:cons ':frameidOriginPosition (frameidOriginPosition msg))
    (cl:cons ':frameidOriginAngle (frameidOriginAngle msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':angleType (angleType msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':velocityType (velocityType msg))
    (cl:cons ':tolerance (tolerance msg))
    (cl:cons ':timeout (timeout msg))
))
