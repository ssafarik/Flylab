; Auto-generated. Do not edit!


(cl:in-package plate_tf-msg)


;//! \htmlinclude InBounds.msg.html

(cl:defclass <InBounds> (roslisp-msg-protocol:ros-message)
  ((bounds_radius
    :reader bounds_radius
    :initarg :bounds_radius
    :type cl:float
    :initform 0.0)
   (robot_in_bounds
    :reader robot_in_bounds
    :initarg :robot_in_bounds
    :type cl:boolean
    :initform cl:nil)
   (fly_in_bounds
    :reader fly_in_bounds
    :initarg :fly_in_bounds
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass InBounds (<InBounds>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InBounds>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InBounds)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plate_tf-msg:<InBounds> is deprecated: use plate_tf-msg:InBounds instead.")))

(cl:ensure-generic-function 'bounds_radius-val :lambda-list '(m))
(cl:defmethod bounds_radius-val ((m <InBounds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plate_tf-msg:bounds_radius-val is deprecated.  Use plate_tf-msg:bounds_radius instead.")
  (bounds_radius m))

(cl:ensure-generic-function 'robot_in_bounds-val :lambda-list '(m))
(cl:defmethod robot_in_bounds-val ((m <InBounds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plate_tf-msg:robot_in_bounds-val is deprecated.  Use plate_tf-msg:robot_in_bounds instead.")
  (robot_in_bounds m))

(cl:ensure-generic-function 'fly_in_bounds-val :lambda-list '(m))
(cl:defmethod fly_in_bounds-val ((m <InBounds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plate_tf-msg:fly_in_bounds-val is deprecated.  Use plate_tf-msg:fly_in_bounds instead.")
  (fly_in_bounds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InBounds>) ostream)
  "Serializes a message object of type '<InBounds>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bounds_radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'robot_in_bounds) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fly_in_bounds) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InBounds>) istream)
  "Deserializes a message object of type '<InBounds>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bounds_radius) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'robot_in_bounds) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fly_in_bounds) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InBounds>)))
  "Returns string type for a message object of type '<InBounds>"
  "plate_tf/InBounds")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InBounds)))
  "Returns string type for a message object of type 'InBounds"
  "plate_tf/InBounds")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InBounds>)))
  "Returns md5sum for a message object of type '<InBounds>"
  "8ec52c16dbacf56e543de6136a368dc1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InBounds)))
  "Returns md5sum for a message object of type 'InBounds"
  "8ec52c16dbacf56e543de6136a368dc1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InBounds>)))
  "Returns full string definition for message of type '<InBounds>"
  (cl:format cl:nil "float64 bounds_radius~%bool robot_in_bounds~%bool fly_in_bounds~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InBounds)))
  "Returns full string definition for message of type 'InBounds"
  (cl:format cl:nil "float64 bounds_radius~%bool robot_in_bounds~%bool fly_in_bounds~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InBounds>))
  (cl:+ 0
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InBounds>))
  "Converts a ROS message object to a list"
  (cl:list 'InBounds
    (cl:cons ':bounds_radius (bounds_radius msg))
    (cl:cons ':robot_in_bounds (robot_in_bounds msg))
    (cl:cons ':fly_in_bounds (fly_in_bounds msg))
))
