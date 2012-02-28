; Auto-generated. Do not edit!


(cl:in-package image_gui-msg)


;//! \htmlinclude CvScalar.msg.html

(cl:defclass <CvScalar> (roslisp-msg-protocol:ros-message)
  ((values
    :reader values
    :initarg :values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CvScalar (<CvScalar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CvScalar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CvScalar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_gui-msg:<CvScalar> is deprecated: use image_gui-msg:CvScalar instead.")))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <CvScalar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:values-val is deprecated.  Use image_gui-msg:values instead.")
  (values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CvScalar>) ostream)
  "Serializes a message object of type '<CvScalar>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'values))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CvScalar>) istream)
  "Deserializes a message object of type '<CvScalar>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CvScalar>)))
  "Returns string type for a message object of type '<CvScalar>"
  "image_gui/CvScalar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CvScalar)))
  "Returns string type for a message object of type 'CvScalar"
  "image_gui/CvScalar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CvScalar>)))
  "Returns md5sum for a message object of type '<CvScalar>"
  "b9163d7c678dcd669317e43e46b63d96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CvScalar)))
  "Returns md5sum for a message object of type 'CvScalar"
  "b9163d7c678dcd669317e43e46b63d96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CvScalar>)))
  "Returns full string definition for message of type '<CvScalar>"
  (cl:format cl:nil "float64[] values~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CvScalar)))
  "Returns full string definition for message of type 'CvScalar"
  (cl:format cl:nil "float64[] values~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CvScalar>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CvScalar>))
  "Converts a ROS message object to a list"
  (cl:list 'CvScalar
    (cl:cons ':values (values msg))
))
