; Auto-generated. Do not edit!


(cl:in-package image_gui-msg)


;//! \htmlinclude CvColor.msg.html

(cl:defclass <CvColor> (roslisp-msg-protocol:ros-message)
  ((red
    :reader red
    :initarg :red
    :type cl:float
    :initform 0.0)
   (green
    :reader green
    :initarg :green
    :type cl:float
    :initform 0.0)
   (blue
    :reader blue
    :initarg :blue
    :type cl:float
    :initform 0.0))
)

(cl:defclass CvColor (<CvColor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CvColor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CvColor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_gui-msg:<CvColor> is deprecated: use image_gui-msg:CvColor instead.")))

(cl:ensure-generic-function 'red-val :lambda-list '(m))
(cl:defmethod red-val ((m <CvColor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:red-val is deprecated.  Use image_gui-msg:red instead.")
  (red m))

(cl:ensure-generic-function 'green-val :lambda-list '(m))
(cl:defmethod green-val ((m <CvColor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:green-val is deprecated.  Use image_gui-msg:green instead.")
  (green m))

(cl:ensure-generic-function 'blue-val :lambda-list '(m))
(cl:defmethod blue-val ((m <CvColor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:blue-val is deprecated.  Use image_gui-msg:blue instead.")
  (blue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CvColor>) ostream)
  "Serializes a message object of type '<CvColor>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'red))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'green))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'blue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CvColor>) istream)
  "Deserializes a message object of type '<CvColor>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'red) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'green) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'blue) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CvColor>)))
  "Returns string type for a message object of type '<CvColor>"
  "image_gui/CvColor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CvColor)))
  "Returns string type for a message object of type 'CvColor"
  "image_gui/CvColor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CvColor>)))
  "Returns md5sum for a message object of type '<CvColor>"
  "0527f4cbd2a07b7bddf52cf785b45c7c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CvColor)))
  "Returns md5sum for a message object of type 'CvColor"
  "0527f4cbd2a07b7bddf52cf785b45c7c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CvColor>)))
  "Returns full string definition for message of type '<CvColor>"
  (cl:format cl:nil "float64 red~%float64 green~%float64 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CvColor)))
  "Returns full string definition for message of type 'CvColor"
  (cl:format cl:nil "float64 red~%float64 green~%float64 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CvColor>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CvColor>))
  "Converts a ROS message object to a list"
  (cl:list 'CvColor
    (cl:cons ':red (red msg))
    (cl:cons ':green (green msg))
    (cl:cons ':blue (blue msg))
))
