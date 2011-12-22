; Auto-generated. Do not edit!


(cl:in-package track_image_contours-msg)


;//! \htmlinclude Stopped.msg.html

(cl:defclass <Stopped> (roslisp-msg-protocol:ros-message)
  ((stopped
    :reader stopped
    :initarg :stopped
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Stopped (<Stopped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stopped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stopped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name track_image_contours-msg:<Stopped> is deprecated: use track_image_contours-msg:Stopped instead.")))

(cl:ensure-generic-function 'stopped-val :lambda-list '(m))
(cl:defmethod stopped-val ((m <Stopped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_image_contours-msg:stopped-val is deprecated.  Use track_image_contours-msg:stopped instead.")
  (stopped m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stopped>) ostream)
  "Serializes a message object of type '<Stopped>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stopped) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stopped>) istream)
  "Deserializes a message object of type '<Stopped>"
    (cl:setf (cl:slot-value msg 'stopped) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stopped>)))
  "Returns string type for a message object of type '<Stopped>"
  "track_image_contours/Stopped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stopped)))
  "Returns string type for a message object of type 'Stopped"
  "track_image_contours/Stopped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stopped>)))
  "Returns md5sum for a message object of type '<Stopped>"
  "caab21a24341159709db962b47b01ad1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stopped)))
  "Returns md5sum for a message object of type 'Stopped"
  "caab21a24341159709db962b47b01ad1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stopped>)))
  "Returns full string definition for message of type '<Stopped>"
  (cl:format cl:nil "bool stopped~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stopped)))
  "Returns full string definition for message of type 'Stopped"
  (cl:format cl:nil "bool stopped~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stopped>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stopped>))
  "Converts a ROS message object to a list"
  (cl:list 'Stopped
    (cl:cons ':stopped (stopped msg))
))
