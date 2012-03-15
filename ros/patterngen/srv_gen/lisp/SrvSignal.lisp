; Auto-generated. Do not edit!


(cl:in-package patterngen-srv)


;//! \htmlinclude SrvSignal-request.msg.html

(cl:defclass <SrvSignal-request> (roslisp-msg-protocol:ros-message)
  ((pts
    :reader pts
    :initarg :pts
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped)))
)

(cl:defclass SrvSignal-request (<SrvSignal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvSignal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvSignal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name patterngen-srv:<SrvSignal-request> is deprecated: use patterngen-srv:SrvSignal-request instead.")))

(cl:ensure-generic-function 'pts-val :lambda-list '(m))
(cl:defmethod pts-val ((m <SrvSignal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader patterngen-srv:pts-val is deprecated.  Use patterngen-srv:pts instead.")
  (pts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvSignal-request>) ostream)
  "Serializes a message object of type '<SrvSignal-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pts) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvSignal-request>) istream)
  "Deserializes a message object of type '<SrvSignal-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pts) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvSignal-request>)))
  "Returns string type for a service object of type '<SrvSignal-request>"
  "patterngen/SrvSignalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvSignal-request)))
  "Returns string type for a service object of type 'SrvSignal-request"
  "patterngen/SrvSignalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvSignal-request>)))
  "Returns md5sum for a message object of type '<SrvSignal-request>"
  "c65831cfc75934a57ad2ea7816766ecf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvSignal-request)))
  "Returns md5sum for a message object of type 'SrvSignal-request"
  "c65831cfc75934a57ad2ea7816766ecf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvSignal-request>)))
  "Returns full string definition for message of type '<SrvSignal-request>"
  (cl:format cl:nil "geometry_msgs/PointStamped pts~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvSignal-request)))
  "Returns full string definition for message of type 'SrvSignal-request"
  (cl:format cl:nil "geometry_msgs/PointStamped pts~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvSignal-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pts))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvSignal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvSignal-request
    (cl:cons ':pts (pts msg))
))
;//! \htmlinclude SrvSignal-response.msg.html

(cl:defclass <SrvSignal-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SrvSignal-response (<SrvSignal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvSignal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvSignal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name patterngen-srv:<SrvSignal-response> is deprecated: use patterngen-srv:SrvSignal-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SrvSignal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader patterngen-srv:success-val is deprecated.  Use patterngen-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvSignal-response>) ostream)
  "Serializes a message object of type '<SrvSignal-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvSignal-response>) istream)
  "Deserializes a message object of type '<SrvSignal-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvSignal-response>)))
  "Returns string type for a service object of type '<SrvSignal-response>"
  "patterngen/SrvSignalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvSignal-response)))
  "Returns string type for a service object of type 'SrvSignal-response"
  "patterngen/SrvSignalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvSignal-response>)))
  "Returns md5sum for a message object of type '<SrvSignal-response>"
  "c65831cfc75934a57ad2ea7816766ecf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvSignal-response)))
  "Returns md5sum for a message object of type 'SrvSignal-response"
  "c65831cfc75934a57ad2ea7816766ecf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvSignal-response>)))
  "Returns full string definition for message of type '<SrvSignal-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvSignal-response)))
  "Returns full string definition for message of type 'SrvSignal-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvSignal-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvSignal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvSignal-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SrvSignal)))
  'SrvSignal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SrvSignal)))
  'SrvSignal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvSignal)))
  "Returns string type for a service object of type '<SrvSignal>"
  "patterngen/SrvSignal")