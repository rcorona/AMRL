; Auto-generated. Do not edit!


(cl:in-package particle_filter-msg)


;//! \htmlinclude Particle.msg.html

(cl:defclass <Particle> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type particle_filter-msg:Pose
    :initform (cl:make-instance 'particle_filter-msg:Pose))
   (weight
    :reader weight
    :initarg :weight
    :type cl:float
    :initform 0.0))
)

(cl:defclass Particle (<Particle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Particle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Particle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name particle_filter-msg:<Particle> is deprecated: use particle_filter-msg:Particle instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Particle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader particle_filter-msg:pose-val is deprecated.  Use particle_filter-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <Particle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader particle_filter-msg:weight-val is deprecated.  Use particle_filter-msg:weight instead.")
  (weight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Particle>) ostream)
  "Serializes a message object of type '<Particle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Particle>) istream)
  "Deserializes a message object of type '<Particle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Particle>)))
  "Returns string type for a message object of type '<Particle>"
  "particle_filter/Particle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Particle)))
  "Returns string type for a message object of type 'Particle"
  "particle_filter/Particle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Particle>)))
  "Returns md5sum for a message object of type '<Particle>"
  "17175abc0cf3cd5dc3269c3b10409ae1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Particle)))
  "Returns md5sum for a message object of type 'Particle"
  "17175abc0cf3cd5dc3269c3b10409ae1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Particle>)))
  "Returns full string definition for message of type '<Particle>"
  (cl:format cl:nil "Pose pose~%float64 weight~%~%================================================================================~%MSG: particle_filter/Pose~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Particle)))
  "Returns full string definition for message of type 'Particle"
  (cl:format cl:nil "Pose pose~%float64 weight~%~%================================================================================~%MSG: particle_filter/Pose~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Particle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Particle>))
  "Converts a ROS message object to a list"
  (cl:list 'Particle
    (cl:cons ':pose (pose msg))
    (cl:cons ':weight (weight msg))
))
