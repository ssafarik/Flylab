; Auto-generated. Do not edit!


(cl:in-package image_gui-msg)


;//! \htmlinclude DrawObjects.msg.html

(cl:defclass <DrawObjects> (roslisp-msg-protocol:ros-message)
  ((show_all
    :reader show_all
    :initarg :show_all
    :type cl:boolean
    :initform cl:nil)
   (hide_all
    :reader hide_all
    :initarg :hide_all
    :type cl:boolean
    :initform cl:nil)
   (draw_object_list
    :reader draw_object_list
    :initarg :draw_object_list
    :type (cl:vector image_gui-msg:DrawObject)
   :initform (cl:make-array 0 :element-type 'image_gui-msg:DrawObject :initial-element (cl:make-instance 'image_gui-msg:DrawObject))))
)

(cl:defclass DrawObjects (<DrawObjects>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrawObjects>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrawObjects)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_gui-msg:<DrawObjects> is deprecated: use image_gui-msg:DrawObjects instead.")))

(cl:ensure-generic-function 'show_all-val :lambda-list '(m))
(cl:defmethod show_all-val ((m <DrawObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:show_all-val is deprecated.  Use image_gui-msg:show_all instead.")
  (show_all m))

(cl:ensure-generic-function 'hide_all-val :lambda-list '(m))
(cl:defmethod hide_all-val ((m <DrawObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:hide_all-val is deprecated.  Use image_gui-msg:hide_all instead.")
  (hide_all m))

(cl:ensure-generic-function 'draw_object_list-val :lambda-list '(m))
(cl:defmethod draw_object_list-val ((m <DrawObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_gui-msg:draw_object_list-val is deprecated.  Use image_gui-msg:draw_object_list instead.")
  (draw_object_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrawObjects>) ostream)
  "Serializes a message object of type '<DrawObjects>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'show_all) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hide_all) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'draw_object_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'draw_object_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrawObjects>) istream)
  "Deserializes a message object of type '<DrawObjects>"
    (cl:setf (cl:slot-value msg 'show_all) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hide_all) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'draw_object_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'draw_object_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'image_gui-msg:DrawObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrawObjects>)))
  "Returns string type for a message object of type '<DrawObjects>"
  "image_gui/DrawObjects")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrawObjects)))
  "Returns string type for a message object of type 'DrawObjects"
  "image_gui/DrawObjects")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrawObjects>)))
  "Returns md5sum for a message object of type '<DrawObjects>"
  "09167aa6575b6803d68a4405eaa911e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrawObjects)))
  "Returns md5sum for a message object of type 'DrawObjects"
  "09167aa6575b6803d68a4405eaa911e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrawObjects>)))
  "Returns full string definition for message of type '<DrawObjects>"
  (cl:format cl:nil "bool show_all~%bool hide_all~%DrawObject[] draw_object_list~%~%================================================================================~%MSG: image_gui/DrawObject~%bool show~%CvPoint object_center~%CvLine[] line_list~%CvCircle[] circle_list~%================================================================================~%MSG: image_gui/CvPoint~%int32 x~%int32 y~%~%================================================================================~%MSG: image_gui/CvLine~%CvPoint point1~%CvPoint point2~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%================================================================================~%MSG: image_gui/CvColor~%float64 red~%float64 green~%float64 blue~%================================================================================~%MSG: image_gui/CvCircle~%CvPoint center~%int32 radius~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrawObjects)))
  "Returns full string definition for message of type 'DrawObjects"
  (cl:format cl:nil "bool show_all~%bool hide_all~%DrawObject[] draw_object_list~%~%================================================================================~%MSG: image_gui/DrawObject~%bool show~%CvPoint object_center~%CvLine[] line_list~%CvCircle[] circle_list~%================================================================================~%MSG: image_gui/CvPoint~%int32 x~%int32 y~%~%================================================================================~%MSG: image_gui/CvLine~%CvPoint point1~%CvPoint point2~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%================================================================================~%MSG: image_gui/CvColor~%float64 red~%float64 green~%float64 blue~%================================================================================~%MSG: image_gui/CvCircle~%CvPoint center~%int32 radius~%CvColor color~%int32 thickness~%int32 lineType~%int32 shift~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrawObjects>))
  (cl:+ 0
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'draw_object_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrawObjects>))
  "Converts a ROS message object to a list"
  (cl:list 'DrawObjects
    (cl:cons ':show_all (show_all msg))
    (cl:cons ':hide_all (hide_all msg))
    (cl:cons ':draw_object_list (draw_object_list msg))
))
