; Auto-generated. Do not edit!


(cl:in-package particle_filter-msg)


;//! \htmlinclude Particle_vector.msg.html

(cl:defclass <Particle_vector> (roslisp-msg-protocol:ros-message)
  ((particles
    :reader particles
    :initarg :particles
    :type (cl:vector particle_filter-msg:Particle)
   :initform (cl:make-array 0 :element-type 'particle_filter-msg:Particle :initial-element (cl:make-instance 'particle_filter-msg:Particle)))
   (pose_estimate
    :reader pose_estimate
    :initarg :pose_estimate
    :type particle_filter-msg:Pose
    :initform (cl:make-instance 'particle_filter-msg:Pose)))
)

(cl:defclass Particle_vector (<Particle_vector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Particle_vector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Particle_vector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name particle_filter-msg:<Particle_vector> is deprecated: use particle_filter-msg:Particle_vector instead.")))

(cl:ensure-generic-function 'particles-val :lambda-list '(m))
(cl:defmethod particles-val ((m <Particle_vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader particle_filter-msg:particles-val is deprecated.  Use particle_filter-msg:particles instead.")
  (particles m))

(cl:ensure-generic-function 'pose_estimate-val :lambda-list '(m))
(cl:defmethod pose_estimate-val ((m <Particle_vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader particle_filter-msg:pose_estimate-val is deprecated.  Use particle_filter-msg:pose_estimate instead.")
  (pose_estimate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Particle_vector>) ostream)
  "Serializes a message object of type '<Particle_vector>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'particles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'particles))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_estimate) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Particle_vector>) istream)
  "Deserializes a message object of type '<Particle_vector>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'particles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'particles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'particle_filter-msg:Particle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_estimate) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Particle_vector>)))
  "Returns string type for a message object of type '<Particle_vector>"
  "particle_filter/Particle_vector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Particle_vector)))
  "Returns string type for a message object of type 'Particle_vector"
  "particle_filter/Particle_vector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Particle_vector>)))
  "Returns md5sum for a message object of type '<Particle_vector>"
  "3ac188ff3c52decc5d6ddfaf7ecb36f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Particle_vector)))
  "Returns md5sum for a message object of type 'Particle_vector"
  "3ac188ff3c52decc5d6ddfaf7ecb36f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Particle_vector>)))
  "Returns full string definition for message of type '<Particle_vector>"
  (cl:format cl:nil "Particle[] particles~%Pose pose_estimate~%~%================================================================================~%MSG: particle_filter/Particle~%Pose pose~%float64 weight~%~%================================================================================~%MSG: particle_filter/Pose~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Particle_vector)))
  "Returns full string definition for message of type 'Particle_vector"
  (cl:format cl:nil "Particle[] particles~%Pose pose_estimate~%~%================================================================================~%MSG: particle_filter/Particle~%Pose pose~%float64 weight~%~%================================================================================~%MSG: particle_filter/Pose~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Particle_vector>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'particles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_estimate))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Particle_vector>))
  "Converts a ROS message object to a list"
  (cl:list 'Particle_vector
    (cl:cons ':particles (particles msg))
    (cl:cons ':pose_estimate (pose_estimate msg))
))
