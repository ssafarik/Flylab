; Auto-generated. Do not edit!


(cl:in-package plate_tf-msg)


;//! \htmlinclude FlyView.msg.html

(cl:defclass <FlyView> (roslisp-msg-protocol:ros-message)
  ((robot_position_x
    :reader robot_position_x
    :initarg :robot_position_x
    :type cl:float
    :initform 0.0)
   (robot_position_y
    :reader robot_position_y
    :initarg :robot_position_y
    :type cl:float
    :initform 0.0)
   (robot_angle
    :reader robot_angle
    :initarg :robot_angle
    :type cl:float
    :initform 0.0)
   (robot_distance
    :reader robot_distance
    :initarg :robot_distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass FlyView (<FlyView>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlyView>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlyView)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plate_tf-msg:<FlyView> is deprecated: use plate_tf-msg:FlyView instead.")))

(cl:ensure-generic-function 'robot_position_x-val :lambda-list '(m))
(cl:defmethod robot_position_x-val ((m <FlyView>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plate_tf-msg:robot_position_x-val is deprecated.  Use plate_tf-msg:robot_position_x instead.")
  (robot_position_x m))

(cl:ensure-generic-function 'robot_position_y-val :lambda-list '(m))
(cl:defmethod robot_position_y-val ((m <FlyView>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plate_tf-msg:robot_position_y-val is deprecated.  Use plate_tf-msg:robot_position_y instead.")
  (robot_position_y m))

(cl:ensure-generic-function 'robot_angle-val :lambda-list '(m))
(cl:defmethod robot_angle-val ((m <FlyView>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plate_tf-msg:robot_angle-val is deprecated.  Use plate_tf-msg:robot_angle instead.")
  (robot_angle m))

(cl:ensure-generic-function 'robot_distance-val :lambda-list '(m))
(cl:defmethod robot_distance-val ((m <FlyView>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plate_tf-msg:robot_distance-val is deprecated.  Use plate_tf-msg:robot_distance instead.")
  (robot_distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlyView>) ostream)
  "Serializes a message object of type '<FlyView>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_position_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_position_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlyView>) istream)
  "Deserializes a message object of type '<FlyView>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_position_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_position_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_distance) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlyView>)))
  "Returns string type for a message object of type '<FlyView>"
  "plate_tf/FlyView")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlyView)))
  "Returns string type for a message object of type 'FlyView"
  "plate_tf/FlyView")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlyView>)))
  "Returns md5sum for a message object of type '<FlyView>"
  "d3b7acfeacb7f132972aed28543a3ffa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlyView)))
  "Returns md5sum for a message object of type 'FlyView"
  "d3b7acfeacb7f132972aed28543a3ffa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlyView>)))
  "Returns full string definition for message of type '<FlyView>"
  (cl:format cl:nil "float64 robot_position_x~%float64 robot_position_y~%float64 robot_angle~%float64 robot_distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlyView)))
  "Returns full string definition for message of type 'FlyView"
  (cl:format cl:nil "float64 robot_position_x~%float64 robot_position_y~%float64 robot_angle~%float64 robot_distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlyView>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlyView>))
  "Converts a ROS message object to a list"
  (cl:list 'FlyView
    (cl:cons ':robot_position_x (robot_position_x msg))
    (cl:cons ':robot_position_y (robot_position_y msg))
    (cl:cons ':robot_angle (robot_angle msg))
    (cl:cons ':robot_distance (robot_distance msg))
))
