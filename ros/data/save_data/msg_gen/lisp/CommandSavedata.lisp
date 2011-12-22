; Auto-generated. Do not edit!


(cl:in-package save_data-msg)


;//! \htmlinclude CommandSavedata.msg.html

(cl:defclass <CommandSavedata> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform "")
   (save_arenastate
    :reader save_arenastate
    :initarg :save_arenastate
    :type cl:boolean
    :initform cl:nil)
   (save_video
    :reader save_video
    :initarg :save_video
    :type cl:boolean
    :initform cl:nil)
   (save_bag
    :reader save_bag
    :initarg :save_bag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CommandSavedata (<CommandSavedata>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandSavedata>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandSavedata)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name save_data-msg:<CommandSavedata> is deprecated: use save_data-msg:CommandSavedata instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <CommandSavedata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:filename-val is deprecated.  Use save_data-msg:filename instead.")
  (filename m))

(cl:ensure-generic-function 'save_arenastate-val :lambda-list '(m))
(cl:defmethod save_arenastate-val ((m <CommandSavedata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:save_arenastate-val is deprecated.  Use save_data-msg:save_arenastate instead.")
  (save_arenastate m))

(cl:ensure-generic-function 'save_video-val :lambda-list '(m))
(cl:defmethod save_video-val ((m <CommandSavedata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:save_video-val is deprecated.  Use save_data-msg:save_video instead.")
  (save_video m))

(cl:ensure-generic-function 'save_bag-val :lambda-list '(m))
(cl:defmethod save_bag-val ((m <CommandSavedata>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:save_bag-val is deprecated.  Use save_data-msg:save_bag instead.")
  (save_bag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandSavedata>) ostream)
  "Serializes a message object of type '<CommandSavedata>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'save_arenastate) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'save_video) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'save_bag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandSavedata>) istream)
  "Deserializes a message object of type '<CommandSavedata>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'save_arenastate) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'save_video) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'save_bag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandSavedata>)))
  "Returns string type for a message object of type '<CommandSavedata>"
  "save_data/CommandSavedata")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandSavedata)))
  "Returns string type for a message object of type 'CommandSavedata"
  "save_data/CommandSavedata")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandSavedata>)))
  "Returns md5sum for a message object of type '<CommandSavedata>"
  "cd82600eeba52d4a8018b7b5c960977a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandSavedata)))
  "Returns md5sum for a message object of type 'CommandSavedata"
  "cd82600eeba52d4a8018b7b5c960977a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandSavedata>)))
  "Returns full string definition for message of type '<CommandSavedata>"
  (cl:format cl:nil "string filename~%bool save_arenastate~%bool save_video~%bool save_bag~%~%#string protocol~%#int32 trial_number~%#float64 angular_velocity_goal~%#bool rm_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandSavedata)))
  "Returns full string definition for message of type 'CommandSavedata"
  (cl:format cl:nil "string filename~%bool save_arenastate~%bool save_video~%bool save_bag~%~%#string protocol~%#int32 trial_number~%#float64 angular_velocity_goal~%#bool rm_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandSavedata>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandSavedata>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandSavedata
    (cl:cons ':filename (filename msg))
    (cl:cons ':save_arenastate (save_arenastate msg))
    (cl:cons ':save_video (save_video msg))
    (cl:cons ':save_bag (save_bag msg))
))
