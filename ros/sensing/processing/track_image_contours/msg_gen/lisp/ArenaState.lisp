; Auto-generated. Do not edit!


(cl:in-package track_image_contours-msg)


;//! \htmlinclude ArenaState.msg.html

(cl:defclass <ArenaState> (roslisp-msg-protocol:ros-message)
  ((robot
    :reader robot
    :initarg :robot
    :type flycore-msg:MsgFrameState
    :initform (cl:make-instance 'flycore-msg:MsgFrameState))
   (flies
    :reader flies
    :initarg :flies
    :type (cl:vector flycore-msg:MsgFrameState)
   :initform (cl:make-array 0 :element-type 'flycore-msg:MsgFrameState :initial-element (cl:make-instance 'flycore-msg:MsgFrameState))))
)

(cl:defclass ArenaState (<ArenaState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArenaState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArenaState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name track_image_contours-msg:<ArenaState> is deprecated: use track_image_contours-msg:ArenaState instead.")))

(cl:ensure-generic-function 'robot-val :lambda-list '(m))
(cl:defmethod robot-val ((m <ArenaState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_image_contours-msg:robot-val is deprecated.  Use track_image_contours-msg:robot instead.")
  (robot m))

(cl:ensure-generic-function 'flies-val :lambda-list '(m))
(cl:defmethod flies-val ((m <ArenaState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_image_contours-msg:flies-val is deprecated.  Use track_image_contours-msg:flies instead.")
  (flies m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArenaState>) ostream)
  "Serializes a message object of type '<ArenaState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'flies))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'flies))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArenaState>) istream)
  "Deserializes a message object of type '<ArenaState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'flies) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'flies)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'flycore-msg:MsgFrameState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArenaState>)))
  "Returns string type for a message object of type '<ArenaState>"
  "track_image_contours/ArenaState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArenaState)))
  "Returns string type for a message object of type 'ArenaState"
  "track_image_contours/ArenaState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArenaState>)))
  "Returns md5sum for a message object of type '<ArenaState>"
  "9f7ee9ebaa8f8ddbe0d1e79f64216360")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArenaState)))
  "Returns md5sum for a message object of type 'ArenaState"
  "9f7ee9ebaa8f8ddbe0d1e79f64216360")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArenaState>)))
  "Returns full string definition for message of type '<ArenaState>"
  (cl:format cl:nil "flycore/MsgFrameState robot~%flycore/MsgFrameState[] flies~%~%~%================================================================================~%MSG: flycore/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArenaState)))
  "Returns full string definition for message of type 'ArenaState"
  (cl:format cl:nil "flycore/MsgFrameState robot~%flycore/MsgFrameState[] flies~%~%~%================================================================================~%MSG: flycore/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArenaState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'flies) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArenaState>))
  "Converts a ROS message object to a list"
  (cl:list 'ArenaState
    (cl:cons ':robot (robot msg))
    (cl:cons ':flies (flies msg))
))
