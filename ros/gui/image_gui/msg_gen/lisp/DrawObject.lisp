; Auto-generated. Do not edit!


(cl:in-package image_gui-msg)


;//! \htmlinclude DrawObject.msg.html

(cl:defclass <DrawObject> (roslisp-msg-protocol:ros-message)
  ((show
    :reader show
    :initarg :show
    :type cl:boolean
    :initform cl:nil)
   (object_center
    :reader object_center
    :initarg :object_center
    :type image_gui-msg:CvPoint
    :initform (cl:make-instance 'image_gui-msg:CvPoint))
   (line_list
    :reader line_list
    :initarg :line_list
    :type (cl:vector image_gui-msg:CvLine)
   :initform (cl:make-array 0 :element-type 'image_gui-msg:CvLine :initial-element (cl:make-instance 'image_gui-msg:CvLine)))
   (circle_list
    :reader circle_list
    :initarg :circle_list
    :type (cl:vector image_gui-msg:CvCircle)
   :initform (cl:make-array 0 :element-type 'image_gui-msg:CvCircle :initial-element (cl:make-instance 'image_gui-msg:CvCircle))))
)

(cl:defclass DrawObject (<DrawObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrawObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrawObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_gui-msg:<DrawObject> is deprecated: use image_gui-msg:DrawObject instead.")))

(cl:ensure-generic-function 'show-val :lambda-list '(m))
(cl:defmethod show-val ((m <DrawObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:show-val is deprecated.  Use image_gui-msg:show instead.")
  (show m))

(cl:ensure-generic-function 'object_center-val :lambda-list '(m))
(cl:defmethod object_center-val ((m <DrawObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:object_center-val is deprecated.  Use image_gui-msg:object_center instead.")
  (object_center m))

(cl:ensure-generic-function 'line_list-val :lambda-list '(m))
(cl:defmethod line_list-val ((m <DrawObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:line_list-val is deprecated.  Use image_gui-msg:line_list instead.")
  (line_list m))

(cl:ensure-generic-function 'circle_list-val :lambda-list '(m))
(cl:defmethod circle_list-val ((m <DrawObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:circle_list-val is deprecated.  Use image_gui-msg:circle_list instead.")
  (circle_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrawObject>) ostream)
  "Serializes a message object of type '<DrawObject>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'show) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_center) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'line_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'line_list))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'circle_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'circle_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrawObject>) istream)
  "Deserializes a message object of type '<DrawObject>"
    (cl:setf (cl:slot-value msg 'show) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_center) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'line_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'line_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'image_gui-msg:CvLine))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'circle_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'circle_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'image_gui-msg:CvCircle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrawObject>)))
  "Returns string type for a message object of type '<DrawObject>"
  "image_gui/DrawObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrawObject)))
  "Returns string type for a message object of type 'DrawObject"
  "image_gui/DrawObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrawObject>)))
  "Returns md5sum for a message object of type '<DrawObject>"
  "2384beb9729e341fbbc183cc2845fddf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrawObject)))
  "Returns md5sum for a message object of type 'DrawObject"
  "2384beb9729e341fbbc183cc2845fddf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrawObject>)))
  "Returns full string definition for message of type '<DrawObject>"
  (cl:format cl:nil "bool show~%CvPoint object_center~%CvLine[] line_list~%CvCircle[] circle_list~%================================================================================~%MSG: image_gui/CvPoint~%int32 x~%int32 y~%~%================================================================================~%MSG: image_gui/CvLine~%CvPoint point1~%CvPoint point2~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%================================================================================~%MSG: image_gui/CvColor~%float64 red~%float64 green~%float64 blue~%================================================================================~%MSG: image_gui/CvCircle~%CvPoint center~%int32 radius~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrawObject)))
  "Returns full string definition for message of type 'DrawObject"
  (cl:format cl:nil "bool show~%CvPoint object_center~%CvLine[] line_list~%CvCircle[] circle_list~%================================================================================~%MSG: image_gui/CvPoint~%int32 x~%int32 y~%~%================================================================================~%MSG: image_gui/CvLine~%CvPoint point1~%CvPoint point2~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%================================================================================~%MSG: image_gui/CvColor~%float64 red~%float64 green~%float64 blue~%================================================================================~%MSG: image_gui/CvCircle~%CvPoint center~%int32 radius~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrawObject>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_center))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'line_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'circle_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrawObject>))
  "Converts a ROS message object to a list"
  (cl:list 'DrawObject
    (cl:cons ':show (show msg))
    (cl:cons ':object_center (object_center msg))
    (cl:cons ':line_list (line_list msg))
    (cl:cons ':circle_list (circle_list msg))
))
