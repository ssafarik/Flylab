; Auto-generated. Do not edit!


(cl:in-package save_data-msg)


;//! \htmlinclude BagInfo.msg.html

(cl:defclass <BagInfo> (roslisp-msg-protocol:ros-message)
  ((bag_name
    :reader bag_name
    :initarg :bag_name
    :type cl:string
    :initform "")
   (ready_to_play
    :reader ready_to_play
    :initarg :ready_to_play
    :type cl:boolean
    :initform cl:nil)
   (finished_playing
    :reader finished_playing
    :initarg :finished_playing
    :type cl:boolean
    :initform cl:nil)
   (end_of_bag_files
    :reader end_of_bag_files
    :initarg :end_of_bag_files
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass BagInfo (<BagInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BagInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BagInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name save_data-msg:<BagInfo> is deprecated: use save_data-msg:BagInfo instead.")))

(cl:ensure-generic-function 'bag_name-val :lambda-list '(m))
(cl:defmethod bag_name-val ((m <BagInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:bag_name-val is deprecated.  Use save_data-msg:bag_name instead.")
  (bag_name m))

(cl:ensure-generic-function 'ready_to_play-val :lambda-list '(m))
(cl:defmethod ready_to_play-val ((m <BagInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:ready_to_play-val is deprecated.  Use save_data-msg:ready_to_play instead.")
  (ready_to_play m))

(cl:ensure-generic-function 'finished_playing-val :lambda-list '(m))
(cl:defmethod finished_playing-val ((m <BagInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:finished_playing-val is deprecated.  Use save_data-msg:finished_playing instead.")
  (finished_playing m))

(cl:ensure-generic-function 'end_of_bag_files-val :lambda-list '(m))
(cl:defmethod end_of_bag_files-val ((m <BagInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:end_of_bag_files-val is deprecated.  Use save_data-msg:end_of_bag_files instead.")
  (end_of_bag_files m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BagInfo>) ostream)
  "Serializes a message object of type '<BagInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'bag_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'bag_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ready_to_play) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finished_playing) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'end_of_bag_files) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BagInfo>) istream)
  "Deserializes a message object of type '<BagInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bag_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'bag_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'ready_to_play) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'finished_playing) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'end_of_bag_files) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BagInfo>)))
  "Returns string type for a message object of type '<BagInfo>"
  "save_data/BagInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BagInfo)))
  "Returns string type for a message object of type 'BagInfo"
  "save_data/BagInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BagInfo>)))
  "Returns md5sum for a message object of type '<BagInfo>"
  "b71b65006fb36de257123959aa9c21a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BagInfo)))
  "Returns md5sum for a message object of type 'BagInfo"
  "b71b65006fb36de257123959aa9c21a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BagInfo>)))
  "Returns full string definition for message of type '<BagInfo>"
  (cl:format cl:nil "string bag_name~%bool ready_to_play~%bool finished_playing~%bool end_of_bag_files~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BagInfo)))
  "Returns full string definition for message of type 'BagInfo"
  (cl:format cl:nil "string bag_name~%bool ready_to_play~%bool finished_playing~%bool end_of_bag_files~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BagInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'bag_name))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BagInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'BagInfo
    (cl:cons ':bag_name (bag_name msg))
    (cl:cons ':ready_to_play (ready_to_play msg))
    (cl:cons ':finished_playing (finished_playing msg))
    (cl:cons ':end_of_bag_files (end_of_bag_files msg))
))
