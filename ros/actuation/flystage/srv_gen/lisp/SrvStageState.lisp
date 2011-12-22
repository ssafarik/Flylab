; Auto-generated. Do not edit!


(cl:in-package flystage-srv)


;//! \htmlinclude SrvStageState-request.msg.html

(cl:defclass <SrvStageState-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type flystage-msg:MsgFrameState
    :initform (cl:make-instance 'flystage-msg:MsgFrameState))
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass SrvStageState-request (<SrvStageState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvStageState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvStageState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flystage-srv:<SrvStageState-request> is deprecated: use flystage-srv:SrvStageState-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SrvStageState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flystage-srv:state-val is deprecated.  Use flystage-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <SrvStageState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flystage-srv:velocity-val is deprecated.  Use flystage-srv:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvStageState-request>) ostream)
  "Serializes a message object of type '<SrvStageState-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvStageState-request>) istream)
  "Deserializes a message object of type '<SrvStageState-request>"
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
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvStageState-request>)))
  "Returns string type for a service object of type '<SrvStageState-request>"
  "flystage/SrvStageStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvStageState-request)))
  "Returns string type for a service object of type 'SrvStageState-request"
  "flystage/SrvStageStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvStageState-request>)))
  "Returns md5sum for a message object of type '<SrvStageState-request>"
  "5ceca4cecd48a67d8d224ba74f927fbe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvStageState-request)))
  "Returns md5sum for a message object of type 'SrvStageState-request"
  "5ceca4cecd48a67d8d224ba74f927fbe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvStageState-request>)))
  "Returns full string definition for message of type '<SrvStageState-request>"
  (cl:format cl:nil "MsgFrameState state~%float64 velocity~%~%================================================================================~%MSG: flystage/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvStageState-request)))
  "Returns full string definition for message of type 'SrvStageState-request"
  (cl:format cl:nil "MsgFrameState state~%float64 velocity~%~%================================================================================~%MSG: flystage/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvStageState-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvStageState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvStageState-request
    (cl:cons ':state (state msg))
    (cl:cons ':velocity (velocity msg))
))
;//! \htmlinclude SrvStageState-response.msg.html

(cl:defclass <SrvStageState-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type flystage-msg:MsgFrameState
    :initform (cl:make-instance 'flystage-msg:MsgFrameState)))
)

(cl:defclass SrvStageState-response (<SrvStageState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvStageState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvStageState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flystage-srv:<SrvStageState-response> is deprecated: use flystage-srv:SrvStageState-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SrvStageState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flystage-srv:state-val is deprecated.  Use flystage-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvStageState-response>) ostream)
  "Serializes a message object of type '<SrvStageState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvStageState-response>) istream)
  "Deserializes a message object of type '<SrvStageState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvStageState-response>)))
  "Returns string type for a service object of type '<SrvStageState-response>"
  "flystage/SrvStageStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvStageState-response)))
  "Returns string type for a service object of type 'SrvStageState-response"
  "flystage/SrvStageStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvStageState-response>)))
  "Returns md5sum for a message object of type '<SrvStageState-response>"
  "5ceca4cecd48a67d8d224ba74f927fbe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvStageState-response)))
  "Returns md5sum for a message object of type 'SrvStageState-response"
  "5ceca4cecd48a67d8d224ba74f927fbe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvStageState-response>)))
  "Returns full string definition for message of type '<SrvStageState-response>"
  (cl:format cl:nil "MsgFrameState state~%~%~%~%================================================================================~%MSG: flystage/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvStageState-response)))
  "Returns full string definition for message of type 'SrvStageState-response"
  (cl:format cl:nil "MsgFrameState state~%~%~%~%================================================================================~%MSG: flystage/MsgFrameState~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist velocity~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvStageState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvStageState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvStageState-response
    (cl:cons ':state (state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SrvStageState)))
  'SrvStageState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SrvStageState)))
  'SrvStageState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvStageState)))
  "Returns string type for a service object of type '<SrvStageState>"
  "flystage/SrvStageState")