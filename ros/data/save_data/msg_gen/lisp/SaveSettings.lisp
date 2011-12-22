; Auto-generated. Do not edit!


(cl:in-package save_data-msg)


;//! \htmlinclude SaveSettings.msg.html

(cl:defclass <SaveSettings> (roslisp-msg-protocol:ros-message)
  ((filenamebase
    :reader filenamebase
    :initarg :filenamebase
    :type cl:string
    :initform "")
   (arenastate
    :reader arenastate
    :initarg :arenastate
    :type cl:boolean
    :initform cl:nil)
   (video
    :reader video
    :initarg :video
    :type cl:boolean
    :initform cl:nil)
   (bag
    :reader bag
    :initarg :bag
    :type cl:boolean
    :initform cl:nil)
   (onlyWhileTriggered
    :reader onlyWhileTriggered
    :initarg :onlyWhileTriggered
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SaveSettings (<SaveSettings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveSettings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveSettings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name save_data-msg:<SaveSettings> is deprecated: use save_data-msg:SaveSettings instead.")))

(cl:ensure-generic-function 'filenamebase-val :lambda-list '(m))
(cl:defmethod filenamebase-val ((m <SaveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:filenamebase-val is deprecated.  Use save_data-msg:filenamebase instead.")
  (filenamebase m))

(cl:ensure-generic-function 'arenastate-val :lambda-list '(m))
(cl:defmethod arenastate-val ((m <SaveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:arenastate-val is deprecated.  Use save_data-msg:arenastate instead.")
  (arenastate m))

(cl:ensure-generic-function 'video-val :lambda-list '(m))
(cl:defmethod video-val ((m <SaveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:video-val is deprecated.  Use save_data-msg:video instead.")
  (video m))

(cl:ensure-generic-function 'bag-val :lambda-list '(m))
(cl:defmethod bag-val ((m <SaveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:bag-val is deprecated.  Use save_data-msg:bag instead.")
  (bag m))

(cl:ensure-generic-function 'onlyWhileTriggered-val :lambda-list '(m))
(cl:defmethod onlyWhileTriggered-val ((m <SaveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:onlyWhileTriggered-val is deprecated.  Use save_data-msg:onlyWhileTriggered instead.")
  (onlyWhileTriggered m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveSettings>) ostream)
  "Serializes a message object of type '<SaveSettings>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filenamebase))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filenamebase))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'arenastate) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'video) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'bag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'onlyWhileTriggered) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveSettings>) istream)
  "Deserializes a message object of type '<SaveSettings>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filenamebase) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filenamebase) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'arenastate) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'video) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'bag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'onlyWhileTriggered) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveSettings>)))
  "Returns string type for a message object of type '<SaveSettings>"
  "save_data/SaveSettings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveSettings)))
  "Returns string type for a message object of type 'SaveSettings"
  "save_data/SaveSettings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveSettings>)))
  "Returns md5sum for a message object of type '<SaveSettings>"
  "8dd39306b182f140b176cdb1eabbcdf0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveSettings)))
  "Returns md5sum for a message object of type 'SaveSettings"
  "8dd39306b182f140b176cdb1eabbcdf0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveSettings>)))
  "Returns full string definition for message of type '<SaveSettings>"
  (cl:format cl:nil "string filenamebase~%bool arenastate~%bool video~%bool bag~%bool onlyWhileTriggered~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveSettings)))
  "Returns full string definition for message of type 'SaveSettings"
  (cl:format cl:nil "string filenamebase~%bool arenastate~%bool video~%bool bag~%bool onlyWhileTriggered~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveSettings>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filenamebase))
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveSettings>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveSettings
    (cl:cons ':filenamebase (filenamebase msg))
    (cl:cons ':arenastate (arenastate msg))
    (cl:cons ':video (video msg))
    (cl:cons ':bag (bag msg))
    (cl:cons ':onlyWhileTriggered (onlyWhileTriggered msg))
))
