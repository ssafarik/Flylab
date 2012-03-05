; Auto-generated. Do not edit!


(cl:in-package experiments-srv)


;//! \htmlinclude ExperimentParams-request.msg.html

(cl:defclass <ExperimentParams-request> (roslisp-msg-protocol:ros-message)
  ((experiment
    :reader experiment
    :initarg :experiment
    :type experiments-msg:ExperimentSettings
    :initform (cl:make-instance 'experiments-msg:ExperimentSettings))
   (home
    :reader home
    :initarg :home
    :type experiments-msg:HomeSettings
    :initform (cl:make-instance 'experiments-msg:HomeSettings))
   (waitEntry
    :reader waitEntry
    :initarg :waitEntry
    :type cl:float
    :initform 0.0)
   (triggerEntry
    :reader triggerEntry
    :initarg :triggerEntry
    :type experiments-msg:TriggerSettings
    :initform (cl:make-instance 'experiments-msg:TriggerSettings))
   (move
    :reader move
    :initarg :move
    :type experiments-msg:MoveSettings
    :initform (cl:make-instance 'experiments-msg:MoveSettings))
   (triggerExit
    :reader triggerExit
    :initarg :triggerExit
    :type experiments-msg:TriggerSettings
    :initform (cl:make-instance 'experiments-msg:TriggerSettings))
   (save
    :reader save
    :initarg :save
    :type experiments-msg:SaveSettings
    :initform (cl:make-instance 'experiments-msg:SaveSettings)))
)

(cl:defclass ExperimentParams-request (<ExperimentParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExperimentParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExperimentParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name experiments-srv:<ExperimentParams-request> is deprecated: use experiments-srv:ExperimentParams-request instead.")))

(cl:ensure-generic-function 'experiment-val :lambda-list '(m))
(cl:defmethod experiment-val ((m <ExperimentParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:experiment-val is deprecated.  Use experiments-srv:experiment instead.")
  (experiment m))

(cl:ensure-generic-function 'home-val :lambda-list '(m))
(cl:defmethod home-val ((m <ExperimentParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:home-val is deprecated.  Use experiments-srv:home instead.")
  (home m))

(cl:ensure-generic-function 'waitEntry-val :lambda-list '(m))
(cl:defmethod waitEntry-val ((m <ExperimentParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:waitEntry-val is deprecated.  Use experiments-srv:waitEntry instead.")
  (waitEntry m))

(cl:ensure-generic-function 'triggerEntry-val :lambda-list '(m))
(cl:defmethod triggerEntry-val ((m <ExperimentParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:triggerEntry-val is deprecated.  Use experiments-srv:triggerEntry instead.")
  (triggerEntry m))

(cl:ensure-generic-function 'move-val :lambda-list '(m))
(cl:defmethod move-val ((m <ExperimentParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:move-val is deprecated.  Use experiments-srv:move instead.")
  (move m))

(cl:ensure-generic-function 'triggerExit-val :lambda-list '(m))
(cl:defmethod triggerExit-val ((m <ExperimentParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:triggerExit-val is deprecated.  Use experiments-srv:triggerExit instead.")
  (triggerExit m))

(cl:ensure-generic-function 'save-val :lambda-list '(m))
(cl:defmethod save-val ((m <ExperimentParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:save-val is deprecated.  Use experiments-srv:save instead.")
  (save m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExperimentParams-request>) ostream)
  "Serializes a message object of type '<ExperimentParams-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'experiment) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'home) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'waitEntry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'triggerEntry) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'move) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'triggerExit) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'save) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExperimentParams-request>) istream)
  "Deserializes a message object of type '<ExperimentParams-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'experiment) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'home) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'waitEntry) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'triggerEntry) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'move) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'triggerExit) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'save) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExperimentParams-request>)))
  "Returns string type for a service object of type '<ExperimentParams-request>"
  "experiments/ExperimentParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExperimentParams-request)))
  "Returns string type for a service object of type 'ExperimentParams-request"
  "experiments/ExperimentParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExperimentParams-request>)))
  "Returns md5sum for a message object of type '<ExperimentParams-request>"
  "9a3ed8f74b7b0897436660f47751e051")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExperimentParams-request)))
  "Returns md5sum for a message object of type 'ExperimentParams-request"
  "9a3ed8f74b7b0897436660f47751e051")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExperimentParams-request>)))
  "Returns full string definition for message of type '<ExperimentParams-request>"
  (cl:format cl:nil "ExperimentSettings experiment~%HomeSettings home~%float64 waitEntry~%TriggerSettings triggerEntry~%MoveSettings move~%TriggerSettings triggerExit~%SaveSettings save~%~%================================================================================~%MSG: experiments/ExperimentSettings~%string description~%int32 maxTrials~%int32 trial ~%~%~%================================================================================~%MSG: experiments/HomeSettings~%bool enabled~%float64 x~%float64 y~%float64 tolerance~%float64 timeout~%~%~%~%================================================================================~%MSG: experiments/TriggerSettings~%bool enabled~%float64 distanceMin~%float64 distanceMax~%float64 speedMin~%float64 speedMax~%float64 angleMin~%float64 angleMax~%string  angleTest~%bool    angleTestBilateral~%float64 timeHold~%float64 timeout~%~%~%~%================================================================================~%MSG: experiments/MoveSettings~%bool enabled~%bool tracking~%string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'~%string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'~%float64 distance~%float64 angle~%string angleType # 'random' or 'constant'~%float64 speed~%string speedType # 'random' or 'constant'~%float64 tolerance~%float64 timeout~%~%~%================================================================================~%MSG: experiments/SaveSettings~%string filenamebase~%bool arenastate~%bool video~%bool bag~%bool onlyWhileTriggered~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExperimentParams-request)))
  "Returns full string definition for message of type 'ExperimentParams-request"
  (cl:format cl:nil "ExperimentSettings experiment~%HomeSettings home~%float64 waitEntry~%TriggerSettings triggerEntry~%MoveSettings move~%TriggerSettings triggerExit~%SaveSettings save~%~%================================================================================~%MSG: experiments/ExperimentSettings~%string description~%int32 maxTrials~%int32 trial ~%~%~%================================================================================~%MSG: experiments/HomeSettings~%bool enabled~%float64 x~%float64 y~%float64 tolerance~%float64 timeout~%~%~%~%================================================================================~%MSG: experiments/TriggerSettings~%bool enabled~%float64 distanceMin~%float64 distanceMax~%float64 speedMin~%float64 speedMax~%float64 angleMin~%float64 angleMax~%string  angleTest~%bool    angleTestBilateral~%float64 timeHold~%float64 timeout~%~%~%~%================================================================================~%MSG: experiments/MoveSettings~%bool enabled~%bool tracking~%string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'~%string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'~%float64 distance~%float64 angle~%string angleType # 'random' or 'constant'~%float64 speed~%string speedType # 'random' or 'constant'~%float64 tolerance~%float64 timeout~%~%~%================================================================================~%MSG: experiments/SaveSettings~%string filenamebase~%bool arenastate~%bool video~%bool bag~%bool onlyWhileTriggered~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExperimentParams-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'experiment))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'home))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'triggerEntry))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'move))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'triggerExit))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'save))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExperimentParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExperimentParams-request
    (cl:cons ':experiment (experiment msg))
    (cl:cons ':home (home msg))
    (cl:cons ':waitEntry (waitEntry msg))
    (cl:cons ':triggerEntry (triggerEntry msg))
    (cl:cons ':move (move msg))
    (cl:cons ':triggerExit (triggerExit msg))
    (cl:cons ':save (save msg))
))
;//! \htmlinclude ExperimentParams-response.msg.html

(cl:defclass <ExperimentParams-response> (roslisp-msg-protocol:ros-message)
  ((succeeded
    :reader succeeded
    :initarg :succeeded
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ExperimentParams-response (<ExperimentParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExperimentParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExperimentParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name experiments-srv:<ExperimentParams-response> is deprecated: use experiments-srv:ExperimentParams-response instead.")))

(cl:ensure-generic-function 'succeeded-val :lambda-list '(m))
(cl:defmethod succeeded-val ((m <ExperimentParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader experiments-srv:succeeded-val is deprecated.  Use experiments-srv:succeeded instead.")
  (succeeded m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExperimentParams-response>) ostream)
  "Serializes a message object of type '<ExperimentParams-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'succeeded) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExperimentParams-response>) istream)
  "Deserializes a message object of type '<ExperimentParams-response>"
    (cl:setf (cl:slot-value msg 'succeeded) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExperimentParams-response>)))
  "Returns string type for a service object of type '<ExperimentParams-response>"
  "experiments/ExperimentParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExperimentParams-response)))
  "Returns string type for a service object of type 'ExperimentParams-response"
  "experiments/ExperimentParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExperimentParams-response>)))
  "Returns md5sum for a message object of type '<ExperimentParams-response>"
  "9a3ed8f74b7b0897436660f47751e051")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExperimentParams-response)))
  "Returns md5sum for a message object of type 'ExperimentParams-response"
  "9a3ed8f74b7b0897436660f47751e051")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExperimentParams-response>)))
  "Returns full string definition for message of type '<ExperimentParams-response>"
  (cl:format cl:nil "bool succeeded~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExperimentParams-response)))
  "Returns full string definition for message of type 'ExperimentParams-response"
  (cl:format cl:nil "bool succeeded~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExperimentParams-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExperimentParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExperimentParams-response
    (cl:cons ':succeeded (succeeded msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExperimentParams)))
  'ExperimentParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExperimentParams)))
  'ExperimentParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExperimentParams)))
  "Returns string type for a service object of type '<ExperimentParams>"
  "experiments/ExperimentParams")