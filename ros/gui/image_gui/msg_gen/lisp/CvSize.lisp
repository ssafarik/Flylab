; Auto-generated. Do not edit!


(cl:in-package image_gui-msg)


;//! \htmlinclude CvSize.msg.html

(cl:defclass <CvSize> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0))
)

(cl:defclass CvSize (<CvSize>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CvSize>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CvSize)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_gui-msg:<CvSize> is deprecated: use image_gui-msg:CvSize instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <CvSize>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:width-val is deprecated.  Use image_gui-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <CvSize>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:height-val is deprecated.  Use image_gui-msg:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CvSize>) ostream)
  "Serializes a message object of type '<CvSize>"
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CvSize>) istream)
  "Deserializes a message object of type '<CvSize>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CvSize>)))
  "Returns string type for a message object of type '<CvSize>"
  "image_gui/CvSize")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CvSize)))
  "Returns string type for a message object of type 'CvSize"
  "image_gui/CvSize")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CvSize>)))
  "Returns md5sum for a message object of type '<CvSize>"
  "a4069c3fe743f7881e847e9b0ab5556a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CvSize)))
  "Returns md5sum for a message object of type 'CvSize"
  "a4069c3fe743f7881e847e9b0ab5556a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CvSize>)))
  "Returns full string definition for message of type '<CvSize>"
  (cl:format cl:nil "int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CvSize)))
  "Returns full string definition for message of type 'CvSize"
  (cl:format cl:nil "int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CvSize>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CvSize>))
  "Converts a ROS message object to a list"
  (cl:list 'CvSize
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
