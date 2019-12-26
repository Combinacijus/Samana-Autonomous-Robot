; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude OdometrySmall.msg.html

(cl:defclass <OdometrySmall> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (delta_ticks1
    :reader delta_ticks1
    :initarg :delta_ticks1
    :type cl:fixnum
    :initform 0)
   (delta_ticks2
    :reader delta_ticks2
    :initarg :delta_ticks2
    :type cl:fixnum
    :initform 0)
   (rps1
    :reader rps1
    :initarg :rps1
    :type cl:float
    :initform 0.0)
   (rps2
    :reader rps2
    :initarg :rps2
    :type cl:float
    :initform 0.0)
   (dt
    :reader dt
    :initarg :dt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass OdometrySmall (<OdometrySmall>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OdometrySmall>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OdometrySmall)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<OdometrySmall> is deprecated: use samana_msgs-msg:OdometrySmall instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:header-val is deprecated.  Use samana_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'delta_ticks1-val :lambda-list '(m))
(cl:defmethod delta_ticks1-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:delta_ticks1-val is deprecated.  Use samana_msgs-msg:delta_ticks1 instead.")
  (delta_ticks1 m))

(cl:ensure-generic-function 'delta_ticks2-val :lambda-list '(m))
(cl:defmethod delta_ticks2-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:delta_ticks2-val is deprecated.  Use samana_msgs-msg:delta_ticks2 instead.")
  (delta_ticks2 m))

(cl:ensure-generic-function 'rps1-val :lambda-list '(m))
(cl:defmethod rps1-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:rps1-val is deprecated.  Use samana_msgs-msg:rps1 instead.")
  (rps1 m))

(cl:ensure-generic-function 'rps2-val :lambda-list '(m))
(cl:defmethod rps2-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:rps2-val is deprecated.  Use samana_msgs-msg:rps2 instead.")
  (rps2 m))

(cl:ensure-generic-function 'dt-val :lambda-list '(m))
(cl:defmethod dt-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dt-val is deprecated.  Use samana_msgs-msg:dt instead.")
  (dt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OdometrySmall>) ostream)
  "Serializes a message object of type '<OdometrySmall>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'delta_ticks1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'delta_ticks2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rps1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rps2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'dt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OdometrySmall>) istream)
  "Deserializes a message object of type '<OdometrySmall>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'delta_ticks1) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'delta_ticks2) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rps1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rps2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dt) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OdometrySmall>)))
  "Returns string type for a message object of type '<OdometrySmall>"
  "samana_msgs/OdometrySmall")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OdometrySmall)))
  "Returns string type for a message object of type 'OdometrySmall"
  "samana_msgs/OdometrySmall")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OdometrySmall>)))
  "Returns md5sum for a message object of type '<OdometrySmall>"
  "98004b29be7f03ed0afbf8488b6e7875")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OdometrySmall)))
  "Returns md5sum for a message object of type 'OdometrySmall"
  "98004b29be7f03ed0afbf8488b6e7875")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OdometrySmall>)))
  "Returns full string definition for message of type '<OdometrySmall>"
  (cl:format cl:nil "std_msgs/Header header~%int16 delta_ticks1~%int16 delta_ticks2~%float32 rps1~%float32 rps2~%int16 dt~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OdometrySmall)))
  "Returns full string definition for message of type 'OdometrySmall"
  (cl:format cl:nil "std_msgs/Header header~%int16 delta_ticks1~%int16 delta_ticks2~%float32 rps1~%float32 rps2~%int16 dt~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OdometrySmall>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     4
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OdometrySmall>))
  "Converts a ROS message object to a list"
  (cl:list 'OdometrySmall
    (cl:cons ':header (header msg))
    (cl:cons ':delta_ticks1 (delta_ticks1 msg))
    (cl:cons ':delta_ticks2 (delta_ticks2 msg))
    (cl:cons ':rps1 (rps1 msg))
    (cl:cons ':rps2 (rps2 msg))
    (cl:cons ':dt (dt msg))
))
