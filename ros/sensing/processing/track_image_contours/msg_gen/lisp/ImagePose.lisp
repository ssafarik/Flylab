; Auto-generated. Do not edit!


(cl:in-package track_image_contours-msg)


;//! \htmlinclude ImagePose.msg.html

(cl:defclass <ImagePose> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (poseRobot
    :reader poseRobot
    :initarg :poseRobot
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (poseFlies
    :reader poseFlies
    :initarg :poseFlies
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass ImagePose (<ImagePose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImagePose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImagePose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name track_image_contours-msg:<ImagePose> is deprecated: use track_image_contours-msg:ImagePose instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ImagePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_image_contours-msg:header-val is deprecated.  Use track_image_contours-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'poseRobot-val :lambda-list '(m))
(cl:defmethod poseRobot-val ((m <ImagePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_image_contours-msg:poseRobot-val is deprecated.  Use track_image_contours-msg:poseRobot instead.")
  (poseRobot m))

(cl:ensure-generic-function 'poseFlies-val :lambda-list '(m))
(cl:defmethod poseFlies-val ((m <ImagePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_image_contours-msg:poseFlies-val is deprecated.  Use track_image_contours-msg:poseFlies instead.")
  (poseFlies m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImagePose>) ostream)
  "Serializes a message object of type '<ImagePose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'poseRobot) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poseFlies))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poseFlies))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImagePose>) istream)
  "Deserializes a message object of type '<ImagePose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'poseRobot) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poseFlies) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poseFlies)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImagePose>)))
  "Returns string type for a message object of type '<ImagePose>"
  "track_image_contours/ImagePose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImagePose)))
  "Returns string type for a message object of type 'ImagePose"
  "track_image_contours/ImagePose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImagePose>)))
  "Returns md5sum for a message object of type '<ImagePose>"
  "f1e7088bef29270f52fc2ad60e564399")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImagePose)))
  "Returns md5sum for a message object of type 'ImagePose"
  "f1e7088bef29270f52fc2ad60e564399")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImagePose>)))
  "Returns full string definition for message of type '<ImagePose>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose poseRobot~%geometry_msgs/Pose[] poseFlies~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImagePose)))
  "Returns full string definition for message of type 'ImagePose"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose poseRobot~%geometry_msgs/Pose[] poseFlies~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImagePose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'poseRobot))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poseFlies) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImagePose>))
  "Converts a ROS message object to a list"
  (cl:list 'ImagePose
    (cl:cons ':header (header msg))
    (cl:cons ':poseRobot (poseRobot msg))
    (cl:cons ':poseFlies (poseFlies msg))
))
