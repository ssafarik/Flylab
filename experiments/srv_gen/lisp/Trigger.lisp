; Auto-generated. Do not edit!


(cl:in-package experiments-srv)


;//! \htmlinclude Trigger-request.msg.html

(cl:defclass <Trigger-request> (roslisp-msg-protocol:ros-message)
  ((triggered
    :reader triggered
    :initarg :triggered
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Trigger-request (<Trigger-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trigger-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trigger-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name experiments-srv:<Trigger-request> is deprecated: use experiments-srv:Trigger-request instead.")))

(cl:ensure-generic-function 'triggered-val :lambda-list '(m))
(cl:defmethod triggered-val ((m <Trigger-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:triggered-val is deprecated.  Use experiments-srv:triggered instead.")
  (triggered m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trigger-request>) ostream)
  "Serializes a message object of type '<Trigger-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'triggered) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trigger-request>) istream)
  "Deserializes a message object of type '<Trigger-request>"
    (cl:setf (cl:slot-value msg 'triggered) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trigger-request>)))
  "Returns string type for a service object of type '<Trigger-request>"
  "experiments/TriggerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trigger-request)))
  "Returns string type for a service object of type 'Trigger-request"
  "experiments/TriggerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trigger-request>)))
  "Returns md5sum for a message object of type '<Trigger-request>"
  "0995ba95fea5a8bded507047fa6164da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trigger-request)))
  "Returns md5sum for a message object of type 'Trigger-request"
  "0995ba95fea5a8bded507047fa6164da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trigger-request>)))
  "Returns full string definition for message of type '<Trigger-request>"
  (cl:format cl:nil "bool triggered~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trigger-request)))
  "Returns full string definition for message of type 'Trigger-request"
  (cl:format cl:nil "bool triggered~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trigger-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trigger-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Trigger-request
    (cl:cons ':triggered (triggered msg))
))
;//! \htmlinclude Trigger-response.msg.html

(cl:defclass <Trigger-response> (roslisp-msg-protocol:ros-message)
  ((triggered
    :reader triggered
    :initarg :triggered
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Trigger-response (<Trigger-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trigger-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trigger-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name experiments-srv:<Trigger-response> is deprecated: use experiments-srv:Trigger-response instead.")))

(cl:ensure-generic-function 'triggered-val :lambda-list '(m))
(cl:defmethod triggered-val ((m <Trigger-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:triggered-val is deprecated.  Use experiments-srv:triggered instead.")
  (triggered m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trigger-response>) ostream)
  "Serializes a message object of type '<Trigger-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'triggered) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trigger-response>) istream)
  "Deserializes a message object of type '<Trigger-response>"
    (cl:setf (cl:slot-value msg 'triggered) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trigger-response>)))
  "Returns string type for a service object of type '<Trigger-response>"
  "experiments/TriggerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trigger-response)))
  "Returns string type for a service object of type 'Trigger-response"
  "experiments/TriggerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trigger-response>)))
  "Returns md5sum for a message object of type '<Trigger-response>"
  "0995ba95fea5a8bded507047fa6164da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trigger-response)))
  "Returns md5sum for a message object of type 'Trigger-response"
  "0995ba95fea5a8bded507047fa6164da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trigger-response>)))
  "Returns full string definition for message of type '<Trigger-response>"
  (cl:format cl:nil "bool triggered~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trigger-response)))
  "Returns full string definition for message of type 'Trigger-response"
  (cl:format cl:nil "bool triggered~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trigger-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trigger-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Trigger-response
    (cl:cons ':triggered (triggered msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Trigger)))
  'Trigger-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Trigger)))
  'Trigger-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trigger)))
  "Returns string type for a service object of type '<Trigger>"
  "experiments/Trigger")