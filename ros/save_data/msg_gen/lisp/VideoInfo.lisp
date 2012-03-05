; Auto-generated. Do not edit!


(cl:in-package save_data-msg)


;//! \htmlinclude VideoInfo.msg.html

(cl:defclass <VideoInfo> (roslisp-msg-protocol:ros-message)
  ((ready_for_bag_info
    :reader ready_for_bag_info
    :initarg :ready_for_bag_info
    :type cl:boolean
    :initform cl:nil)
   (ready_to_record
    :reader ready_to_record
    :initarg :ready_to_record
    :type cl:boolean
    :initform cl:nil)
   (saved_video
    :reader saved_video
    :initarg :saved_video
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VideoInfo (<VideoInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VideoInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VideoInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name save_data-msg:<VideoInfo> is deprecated: use save_data-msg:VideoInfo instead.")))

(cl:ensure-generic-function 'ready_for_bag_info-val :lambda-list '(m))
(cl:defmethod ready_for_bag_info-val ((m <VideoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:ready_for_bag_info-val is deprecated.  Use save_data-msg:ready_for_bag_info instead.")
  (ready_for_bag_info m))

(cl:ensure-generic-function 'ready_to_record-val :lambda-list '(m))
(cl:defmethod ready_to_record-val ((m <VideoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:ready_to_record-val is deprecated.  Use save_data-msg:ready_to_record instead.")
  (ready_to_record m))

(cl:ensure-generic-function 'saved_video-val :lambda-list '(m))
(cl:defmethod saved_video-val ((m <VideoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader save_data-msg:saved_video-val is deprecated.  Use save_data-msg:saved_video instead.")
  (saved_video m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VideoInfo>) ostream)
  "Serializes a message object of type '<VideoInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ready_for_bag_info) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ready_to_record) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'saved_video) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VideoInfo>) istream)
  "Deserializes a message object of type '<VideoInfo>"
    (cl:setf (cl:slot-value msg 'ready_for_bag_info) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ready_to_record) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'saved_video) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VideoInfo>)))
  "Returns string type for a message object of type '<VideoInfo>"
  "save_data/VideoInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VideoInfo)))
  "Returns string type for a message object of type 'VideoInfo"
  "save_data/VideoInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VideoInfo>)))
  "Returns md5sum for a message object of type '<VideoInfo>"
  "8b0e55e78ec597bdfc54926b344bac82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VideoInfo)))
  "Returns md5sum for a message object of type 'VideoInfo"
  "8b0e55e78ec597bdfc54926b344bac82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VideoInfo>)))
  "Returns full string definition for message of type '<VideoInfo>"
  (cl:format cl:nil "bool ready_for_bag_info~%bool ready_to_record~%bool saved_video~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VideoInfo)))
  "Returns full string definition for message of type 'VideoInfo"
  (cl:format cl:nil "bool ready_for_bag_info~%bool ready_to_record~%bool saved_video~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VideoInfo>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VideoInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'VideoInfo
    (cl:cons ':ready_for_bag_info (ready_for_bag_info msg))
    (cl:cons ':ready_to_record (ready_to_record msg))
    (cl:cons ':saved_video (saved_video msg))
))
