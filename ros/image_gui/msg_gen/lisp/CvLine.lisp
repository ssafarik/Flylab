; Auto-generated. Do not edit!


(cl:in-package image_gui-msg)


;//! \htmlinclude CvLine.msg.html

(cl:defclass <CvLine> (roslisp-msg-protocol:ros-message)
  ((point1
    :reader point1
    :initarg :point1
    :type image_gui-msg:CvPoint
    :initform (cl:make-instance 'image_gui-msg:CvPoint))
   (point2
    :reader point2
    :initarg :point2
    :type image_gui-msg:CvPoint
    :initform (cl:make-instance 'image_gui-msg:CvPoint))
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

(cl:defclass CvLine (<CvLine>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CvLine>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CvLine)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_gui-msg:<CvLine> is deprecated: use image_gui-msg:CvLine instead.")))

(cl:ensure-generic-function 'point1-val :lambda-list '(m))
(cl:defmethod point1-val ((m <CvLine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:point1-val is deprecated.  Use image_gui-msg:point1 instead.")
  (point1 m))

(cl:ensure-generic-function 'point2-val :lambda-list '(m))
(cl:defmethod point2-val ((m <CvLine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:point2-val is deprecated.  Use image_gui-msg:point2 instead.")
  (point2 m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <CvLine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:color-val is deprecated.  Use image_gui-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'thickness-val :lambda-list '(m))
(cl:defmethod thickness-val ((m <CvLine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:thickness-val is deprecated.  Use image_gui-msg:thickness instead.")
  (thickness m))

(cl:ensure-generic-function 'lineType-val :lambda-list '(m))
(cl:defmethod lineType-val ((m <CvLine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:lineType-val is deprecated.  Use image_gui-msg:lineType instead.")
  (lineType m))

(cl:ensure-generic-function 'shift-val :lambda-list '(m))
(cl:defmethod shift-val ((m <CvLine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:shift-val is deprecated.  Use image_gui-msg:shift instead.")
  (shift m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CvLine>) ostream)
  "Serializes a message object of type '<CvLine>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point2) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CvLine>) istream)
  "Deserializes a message object of type '<CvLine>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point2) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CvLine>)))
  "Returns string type for a message object of type '<CvLine>"
  "image_gui/CvLine")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CvLine)))
  "Returns string type for a message object of type 'CvLine"
  "image_gui/CvLine")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CvLine>)))
  "Returns md5sum for a message object of type '<CvLine>"
  "f89f52140f0655b9b94e838f7024a03f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CvLine)))
  "Returns md5sum for a message object of type 'CvLine"
  "f89f52140f0655b9b94e838f7024a03f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CvLine>)))
  "Returns full string definition for message of type '<CvLine>"
  (cl:format cl:nil "CvPoint point1~%CvPoint point2~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%================================================================================~%MSG: image_gui/CvPoint~%int32 x~%int32 y~%~%================================================================================~%MSG: image_gui/CvColor~%float64 red~%float64 green~%float64 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CvLine)))
  "Returns full string definition for message of type 'CvLine"
  (cl:format cl:nil "CvPoint point1~%CvPoint point2~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%================================================================================~%MSG: image_gui/CvPoint~%int32 x~%int32 y~%~%================================================================================~%MSG: image_gui/CvColor~%float64 red~%float64 green~%float64 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CvLine>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'color))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CvLine>))
  "Converts a ROS message object to a list"
  (cl:list 'CvLine
    (cl:cons ':point1 (point1 msg))
    (cl:cons ':point2 (point2 msg))
    (cl:cons ':color (color msg))
    (cl:cons ':thickness (thickness msg))
    (cl:cons ':lineType (lineType msg))
    (cl:cons ':shift (shift msg))
))
