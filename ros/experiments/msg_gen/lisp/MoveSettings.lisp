; Auto-generated. Do not edit!


(cl:in-package experiments-msg)


;//! \htmlinclude MoveSettings.msg.html

(cl:defclass <MoveSettings> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil)
   (mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform "")
   (relative
    :reader relative
    :initarg :relative
    :type experiments-msg:MoveRelative
    :initform (cl:make-instance 'experiments-msg:MoveRelative))
   (pattern
    :reader pattern
    :initarg :pattern
    :type experiments-msg:MovePattern
    :initform (cl:make-instance 'experiments-msg:MovePattern))
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

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:mode-val is deprecated.  Use experiments-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'relative-val :lambda-list '(m))
(cl:defmethod relative-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:relative-val is deprecated.  Use experiments-msg:relative instead.")
  (relative m))

(cl:ensure-generic-function 'pattern-val :lambda-list '(m))
(cl:defmethod pattern-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:pattern-val is deprecated.  Use experiments-msg:pattern instead.")
  (pattern m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <MoveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:timeout-val is deprecated.  Use experiments-msg:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveSettings>) ostream)
  "Serializes a message object of type '<MoveSettings>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'relative) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pattern) ostream)
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'relative) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pattern) istream)
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
  "1134374486ede5b193275bb436ea75fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveSettings)))
  "Returns md5sum for a message object of type 'MoveSettings"
  "1134374486ede5b193275bb436ea75fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveSettings>)))
  "Returns full string definition for message of type '<MoveSettings>"
  (cl:format cl:nil "bool enabled~%string mode  # 'pattern' or 'relative'~%MoveRelative relative~%MovePattern pattern~%float64 timeout~%~%~%================================================================================~%MSG: experiments/MoveRelative~%bool tracking~%string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'~%string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'~%float64 distance~%float64 angle~%string angleType # 'random' or 'constant'~%float64 speed~%string speedType # 'random' or 'constant'~%float64 tolerance~%~%~%================================================================================~%MSG: experiments/MovePattern~%string shape  # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral'~%float64 hz~%int32 count  # -1 means forever~%float64 radius~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveSettings)))
  "Returns full string definition for message of type 'MoveSettings"
  (cl:format cl:nil "bool enabled~%string mode  # 'pattern' or 'relative'~%MoveRelative relative~%MovePattern pattern~%float64 timeout~%~%~%================================================================================~%MSG: experiments/MoveRelative~%bool tracking~%string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'~%string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'~%float64 distance~%float64 angle~%string angleType # 'random' or 'constant'~%float64 speed~%string speedType # 'random' or 'constant'~%float64 tolerance~%~%~%================================================================================~%MSG: experiments/MovePattern~%string shape  # 'constant' or 'circle' or 'square' or 'flylogo' or 'spiral'~%float64 hz~%int32 count  # -1 means forever~%float64 radius~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveSettings>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'relative))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pattern))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveSettings>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveSettings
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':relative (relative msg))
    (cl:cons ':pattern (pattern msg))
    (cl:cons ':timeout (timeout msg))
))
