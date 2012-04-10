; Auto-generated. Do not edit!


(cl:in-package experiments-msg)


;//! \htmlinclude TriggerSettings.msg.html

(cl:defclass <TriggerSettings> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil)
   (distanceMin
    :reader distanceMin
    :initarg :distanceMin
    :type cl:float
    :initform 0.0)
   (distanceMax
    :reader distanceMax
    :initarg :distanceMax
    :type cl:float
    :initform 0.0)
   (speedMin
    :reader speedMin
    :initarg :speedMin
    :type cl:float
    :initform 0.0)
   (speedMax
    :reader speedMax
    :initarg :speedMax
    :type cl:float
    :initform 0.0)
   (angleMin
    :reader angleMin
    :initarg :angleMin
    :type cl:float
    :initform 0.0)
   (angleMax
    :reader angleMax
    :initarg :angleMax
    :type cl:float
    :initform 0.0)
   (angleTest
    :reader angleTest
    :initarg :angleTest
    :type cl:string
    :initform "")
   (angleTestBilateral
    :reader angleTestBilateral
    :initarg :angleTestBilateral
    :type cl:boolean
    :initform cl:nil)
   (timeHold
    :reader timeHold
    :initarg :timeHold
    :type cl:float
    :initform 0.0)
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:float
    :initform 0.0))
)

(cl:defclass TriggerSettings (<TriggerSettings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriggerSettings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriggerSettings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name experiments-msg:<TriggerSettings> is deprecated: use experiments-msg:TriggerSettings instead.")))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:enabled-val is deprecated.  Use experiments-msg:enabled instead.")
  (enabled m))

(cl:ensure-generic-function 'distanceMin-val :lambda-list '(m))
(cl:defmethod distanceMin-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:distanceMin-val is deprecated.  Use experiments-msg:distanceMin instead.")
  (distanceMin m))

(cl:ensure-generic-function 'distanceMax-val :lambda-list '(m))
(cl:defmethod distanceMax-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:distanceMax-val is deprecated.  Use experiments-msg:distanceMax instead.")
  (distanceMax m))

(cl:ensure-generic-function 'speedMin-val :lambda-list '(m))
(cl:defmethod speedMin-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:speedMin-val is deprecated.  Use experiments-msg:speedMin instead.")
  (speedMin m))

(cl:ensure-generic-function 'speedMax-val :lambda-list '(m))
(cl:defmethod speedMax-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:speedMax-val is deprecated.  Use experiments-msg:speedMax instead.")
  (speedMax m))

(cl:ensure-generic-function 'angleMin-val :lambda-list '(m))
(cl:defmethod angleMin-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:angleMin-val is deprecated.  Use experiments-msg:angleMin instead.")
  (angleMin m))

(cl:ensure-generic-function 'angleMax-val :lambda-list '(m))
(cl:defmethod angleMax-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:angleMax-val is deprecated.  Use experiments-msg:angleMax instead.")
  (angleMax m))

(cl:ensure-generic-function 'angleTest-val :lambda-list '(m))
(cl:defmethod angleTest-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:angleTest-val is deprecated.  Use experiments-msg:angleTest instead.")
  (angleTest m))

(cl:ensure-generic-function 'angleTestBilateral-val :lambda-list '(m))
(cl:defmethod angleTestBilateral-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:angleTestBilateral-val is deprecated.  Use experiments-msg:angleTestBilateral instead.")
  (angleTestBilateral m))

(cl:ensure-generic-function 'timeHold-val :lambda-list '(m))
(cl:defmethod timeHold-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:timeHold-val is deprecated.  Use experiments-msg:timeHold instead.")
  (timeHold m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <TriggerSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:timeout-val is deprecated.  Use experiments-msg:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriggerSettings>) ostream)
  "Serializes a message object of type '<TriggerSettings>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distanceMin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distanceMax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speedMin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speedMax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angleMin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angleMax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'angleTest))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'angleTest))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'angleTestBilateral) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timeHold))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriggerSettings>) istream)
  "Deserializes a message object of type '<TriggerSettings>"
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distanceMin) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distanceMax) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speedMin) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speedMax) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angleMin) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angleMax) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angleTest) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'angleTest) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'angleTestBilateral) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeHold) (roslisp-utils:decode-double-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriggerSettings>)))
  "Returns string type for a message object of type '<TriggerSettings>"
  "experiments/TriggerSettings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerSettings)))
  "Returns string type for a message object of type 'TriggerSettings"
  "experiments/TriggerSettings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriggerSettings>)))
  "Returns md5sum for a message object of type '<TriggerSettings>"
  "009cb593fc0cecc966de010b763b71c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriggerSettings)))
  "Returns md5sum for a message object of type 'TriggerSettings"
  "009cb593fc0cecc966de010b763b71c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriggerSettings>)))
  "Returns full string definition for message of type '<TriggerSettings>"
  (cl:format cl:nil "bool enabled~%float64 distanceMin~%float64 distanceMax~%float64 speedMin~%float64 speedMax~%float64 angleMin~%float64 angleMax~%string  angleTest~%bool    angleTestBilateral~%float64 timeHold~%float64 timeout~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriggerSettings)))
  "Returns full string definition for message of type 'TriggerSettings"
  (cl:format cl:nil "bool enabled~%float64 distanceMin~%float64 distanceMax~%float64 speedMin~%float64 speedMax~%float64 angleMin~%float64 angleMax~%string  angleTest~%bool    angleTestBilateral~%float64 timeHold~%float64 timeout~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriggerSettings>))
  (cl:+ 0
     1
     8
     8
     8
     8
     8
     8
     4 (cl:length (cl:slot-value msg 'angleTest))
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriggerSettings>))
  "Converts a ROS message object to a list"
  (cl:list 'TriggerSettings
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':distanceMin (distanceMin msg))
    (cl:cons ':distanceMax (distanceMax msg))
    (cl:cons ':speedMin (speedMin msg))
    (cl:cons ':speedMax (speedMax msg))
    (cl:cons ':angleMin (angleMin msg))
    (cl:cons ':angleMax (angleMax msg))
    (cl:cons ':angleTest (angleTest msg))
    (cl:cons ':angleTestBilateral (angleTestBilateral msg))
    (cl:cons ':timeHold (timeHold msg))
    (cl:cons ':timeout (timeout msg))
))
