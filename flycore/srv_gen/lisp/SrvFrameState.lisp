; Auto-generated. Do not edit!


(cl:in-package flycore-srv)


;//! \htmlinclude SrvFrameState-request.msg.html

(cl:defclass <SrvFrameState-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type flycore-msg:MsgFrameState
    :initform (cl:make-instance 'flycore-msg:MsgFrameState))
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass SrvFrameState-request (<SrvFrameState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvFrameState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvFrameState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flycore-srv:<SrvFrameState-request> is deprecated: use flycore-srv:SrvFrameState-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SrvFrameState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flycore-srv:state-val is deprecated.  Use flycore-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <SrvFrameState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flycore-srv:speed-val is deprecated.  Use flycore-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvFrameState-request>) ostream)
  "Serializes a message object of type '<SrvFrameState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvFrameState-request>) istream)
  "Deserializes a message object of type '<SrvFrameState-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvFrameState-request>)))
  "Returns string type for a service object of type '<SrvFrameState-request>"
  "flycore/SrvFrameStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvFrameState-request)))
  "Returns string type for a service object of type 'SrvFrameState-request"
  "flycore/SrvFrameStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvFrameState-request>)))
  "Returns md5sum for a message object of type '<SrvFrameState-request>"
  "a77943b8f87f5da957e62998e6d39b03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvFrameState-request)))
  "Returns md5sum for a message object of type 'SrvFrameState-request"
  "a77943b8f87f5da957e62998e6d39b03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvFrameState-request>)))
  "Returns full string definition for message of type '<SrvFrameState-request>"
  (cl:format cl:nil "MsgFrameState state~%float64 speed~%~%================================================================================~%MSG: flycore/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvFrameState-request)))
  "Returns full string definition for message of type 'SrvFrameState-request"
  (cl:format cl:nil "MsgFrameState state~%float64 speed~%~%================================================================================~%MSG: flycore/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvFrameState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvFrameState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvFrameState-request
    (cl:cons ':state (state msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude SrvFrameState-response.msg.html

(cl:defclass <SrvFrameState-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type flycore-msg:MsgFrameState
    :initform (cl:make-instance 'flycore-msg:MsgFrameState)))
)

(cl:defclass SrvFrameState-response (<SrvFrameState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvFrameState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvFrameState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flycore-srv:<SrvFrameState-response> is deprecated: use flycore-srv:SrvFrameState-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SrvFrameState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flycore-srv:state-val is deprecated.  Use flycore-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvFrameState-response>) ostream)
  "Serializes a message object of type '<SrvFrameState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvFrameState-response>) istream)
  "Deserializes a message object of type '<SrvFrameState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvFrameState-response>)))
  "Returns string type for a service object of type '<SrvFrameState-response>"
  "flycore/SrvFrameStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvFrameState-response)))
  "Returns string type for a service object of type 'SrvFrameState-response"
  "flycore/SrvFrameStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvFrameState-response>)))
  "Returns md5sum for a message object of type '<SrvFrameState-response>"
  "a77943b8f87f5da957e62998e6d39b03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvFrameState-response)))
  "Returns md5sum for a message object of type 'SrvFrameState-response"
  "a77943b8f87f5da957e62998e6d39b03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvFrameState-response>)))
  "Returns full string definition for message of type '<SrvFrameState-response>"
  (cl:format cl:nil "MsgFrameState state~%~%~%~%================================================================================~%MSG: flycore/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvFrameState-response)))
  "Returns full string definition for message of type 'SrvFrameState-response"
  (cl:format cl:nil "MsgFrameState state~%~%~%~%================================================================================~%MSG: flycore/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvFrameState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvFrameState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvFrameState-response
    (cl:cons ':state (state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SrvFrameState)))
  'SrvFrameState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SrvFrameState)))
  'SrvFrameState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvFrameState)))
  "Returns string type for a service object of type '<SrvFrameState>"
  "flycore/SrvFrameState")