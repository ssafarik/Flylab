; Auto-generated. Do not edit!


(cl:in-package image_gui-msg)


;//! \htmlinclude CvCircle.msg.html

(cl:defclass <CvCircle> (roslisp-msg-protocol:ros-message)
  ((center
    :reader center
    :initarg :center
    :type image_gui-msg:CvPoint
    :initform (cl:make-instance 'image_gui-msg:CvPoint))
   (radius
    :reader radius
    :initarg :radius
    :type cl:integer
    :initform 0)
   (color
    :reader color
    :initarg :color
    :type image_gui-msg:CvColor
    :initform (cl:make-instance 'image_gui-msg:CvColor))
   (thickness
    :reader thickness
    :initarg :thickness
    :type cl:integer
    :initform 0)
   (lineType
    :reader lineType
    :initarg :lineType
    :type cl:integer
    :initform 0)
   (shift
    :reader shift
    :initarg :shift
    :type cl:integer
    :initform 0))
)

(cl:defclass CvCircle (<CvCircle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CvCircle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CvCircle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_gui-msg:<CvCircle> is deprecated: use image_gui-msg:CvCircle instead.")))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <CvCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:center-val is deprecated.  Use image_gui-msg:center instead.")
  (center m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <CvCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:radius-val is deprecated.  Use image_gui-msg:radius instead.")
  (radius m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <CvCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:color-val is deprecated.  Use image_gui-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'thickness-val :lambda-list '(m))
(cl:defmethod thickness-val ((m <CvCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:thickness-val is deprecated.  Use image_gui-msg:thickness instead.")
  (thickness m))

(cl:ensure-generic-function 'lineType-val :lambda-list '(m))
(cl:defmethod lineType-val ((m <CvCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:lineType-val is deprecated.  Use image_gui-msg:lineType instead.")
  (lineType m))

(cl:ensure-generic-function 'shift-val :lambda-list '(m))
(cl:defmethod shift-val ((m <CvCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:shift-val is deprecated.  Use image_gui-msg:shift instead.")
  (shift m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CvCircle>) ostream)
  "Serializes a message object of type '<CvCircle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (cl:let* ((signed (cl:slot-value msg 'radius)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'color) ostream)
  (cl:let* ((signed (cl:slot-value msg 'thickness)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lineType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'shift)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CvCircle>) istream)
  "Deserializes a message object of type '<CvCircle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'radius) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'color) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'thickness) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lineType) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'shift) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CvCircle>)))
  "Returns string type for a message object of type '<CvCircle>"
  "image_gui/CvCircle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CvCircle)))
  "Returns string type for a message object of type 'CvCircle"
  "image_gui/CvCircle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CvCircle>)))
  "Returns md5sum for a message object of type '<CvCircle>"
  "a60e9679b4eefa3cfeca04ca12c0783c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CvCircle)))
  "Returns md5sum for a message object of type 'CvCircle"
  "a60e9679b4eefa3cfeca04ca12c0783c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CvCircle>)))
  "Returns full string definition for message of type '<CvCircle>"
  (cl:format cl:nil "CvPoint center~%int32 radius~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%================================================================================~%MSG: image_gui/CvPoint~%int32 x~%int32 y~%~%================================================================================~%MSG: image_gui/CvColor~%float64 red~%float64 green~%float64 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CvCircle)))
  "Returns full string definition for message of type 'CvCircle"
  (cl:format cl:nil "CvPoint center~%int32 radius~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%================================================================================~%MSG: image_gui/CvPoint~%int32 x~%int32 y~%~%================================================================================~%MSG: image_gui/CvColor~%float64 red~%float64 green~%float64 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CvCircle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'color))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CvCircle>))
  "Converts a ROS message object to a list"
  (cl:list 'CvCircle
    (cl:cons ':center (center msg))
    (cl:cons ':radius (radius msg))
    (cl:cons ':color (color msg))
    (cl:cons ':thickness (thickness msg))
    (cl:cons ':lineType (lineType msg))
    (cl:cons ':shift (shift msg))
))
