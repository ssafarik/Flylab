; Auto-generated. Do not edit!


(cl:in-package experiments-msg)


;//! \htmlinclude ExperimentSettings.msg.html

(cl:defclass <ExperimentSettings> (roslisp-msg-protocol:ros-message)
  ((description
    :reader description
    :initarg :description
    :type cl:string
    :initform "")
   (maxTrials
    :reader maxTrials
    :initarg :maxTrials
    :type cl:integer
    :initform 0)
   (trial
    :reader trial
    :initarg :trial
    :type cl:integer
    :initform 0))
)

(cl:defclass ExperimentSettings (<ExperimentSettings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExperimentSettings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExperimentSettings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name experiments-msg:<ExperimentSettings> is deprecated: use experiments-msg:ExperimentSettings instead.")))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <ExperimentSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:description-val is deprecated.  Use experiments-msg:description instead.")
  (description m))

(cl:ensure-generic-function 'maxTrials-val :lambda-list '(m))
(cl:defmethod maxTrials-val ((m <ExperimentSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:maxTrials-val is deprecated.  Use experiments-msg:maxTrials instead.")
  (maxTrials m))

(cl:ensure-generic-function 'trial-val :lambda-list '(m))
(cl:defmethod trial-val ((m <ExperimentSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-msg:trial-val is deprecated.  Use experiments-msg:trial instead.")
  (trial m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExperimentSettings>) ostream)
  "Serializes a message object of type '<ExperimentSettings>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
  (cl:let* ((signed (cl:slot-value msg 'maxTrials)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'trial)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExperimentSettings>) istream)
  "Deserializes a message object of type '<ExperimentSettings>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'maxTrials) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'trial) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExperimentSettings>)))
  "Returns string type for a message object of type '<ExperimentSettings>"
  "experiments/ExperimentSettings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExperimentSettings)))
  "Returns string type for a message object of type 'ExperimentSettings"
  "experiments/ExperimentSettings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExperimentSettings>)))
  "Returns md5sum for a message object of type '<ExperimentSettings>"
  "42e02677c518fd77f77d9cd10747bc20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExperimentSettings)))
  "Returns md5sum for a message object of type 'ExperimentSettings"
  "42e02677c518fd77f77d9cd10747bc20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExperimentSettings>)))
  "Returns full string definition for message of type '<ExperimentSettings>"
  (cl:format cl:nil "string description~%int32 maxTrials~%int32 trial ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExperimentSettings)))
  "Returns full string definition for message of type 'ExperimentSettings"
  (cl:format cl:nil "string description~%int32 maxTrials~%int32 trial ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExperimentSettings>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'description))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExperimentSettings>))
  "Converts a ROS message object to a list"
  (cl:list 'ExperimentSettings
    (cl:cons ':description (description msg))
    (cl:cons ':maxTrials (maxTrials msg))
    (cl:cons ':trial (trial msg))
))
