; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude OdometrySmall.msg.html

(cl:defclass <OdometrySmall> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ticks1
    :reader ticks1
    :initarg :ticks1
    :type cl:fixnum
    :initform 0)
   (ticks2
    :reader ticks2
    :initarg :ticks2
    :type cl:fixnum
    :initform 0)
   (speed1
    :reader speed1
    :initarg :speed1
    :type cl:float
    :initform 0.0)
   (speed2
    :reader speed2
    :initarg :speed2
    :type cl:float
    :initform 0.0))
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

(cl:ensure-generic-function 'ticks1-val :lambda-list '(m))
(cl:defmethod ticks1-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:ticks1-val is deprecated.  Use samana_msgs-msg:ticks1 instead.")
  (ticks1 m))

(cl:ensure-generic-function 'ticks2-val :lambda-list '(m))
(cl:defmethod ticks2-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:ticks2-val is deprecated.  Use samana_msgs-msg:ticks2 instead.")
  (ticks2 m))

(cl:ensure-generic-function 'speed1-val :lambda-list '(m))
(cl:defmethod speed1-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:speed1-val is deprecated.  Use samana_msgs-msg:speed1 instead.")
  (speed1 m))

(cl:ensure-generic-function 'speed2-val :lambda-list '(m))
(cl:defmethod speed2-val ((m <OdometrySmall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:speed2-val is deprecated.  Use samana_msgs-msg:speed2 instead.")
  (speed2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OdometrySmall>) ostream)
  "Serializes a message object of type '<OdometrySmall>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ticks1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ticks1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ticks2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ticks2)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OdometrySmall>) istream)
  "Deserializes a message object of type '<OdometrySmall>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ticks1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ticks1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ticks2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ticks2)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed2) (roslisp-utils:decode-single-float-bits bits)))
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
  "66d5c83f5380a539385185906d186ae9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OdometrySmall)))
  "Returns md5sum for a message object of type 'OdometrySmall"
  "66d5c83f5380a539385185906d186ae9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OdometrySmall>)))
  "Returns full string definition for message of type '<OdometrySmall>"
  (cl:format cl:nil "std_msgs/Header header~%uint16 ticks1~%uint16 ticks2~%float32 speed1~%float32 speed2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OdometrySmall)))
  "Returns full string definition for message of type 'OdometrySmall"
  (cl:format cl:nil "std_msgs/Header header~%uint16 ticks1~%uint16 ticks2~%float32 speed1~%float32 speed2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OdometrySmall>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OdometrySmall>))
  "Converts a ROS message object to a list"
  (cl:list 'OdometrySmall
    (cl:cons ':header (header msg))
    (cl:cons ':ticks1 (ticks1 msg))
    (cl:cons ':ticks2 (ticks2 msg))
    (cl:cons ':speed1 (speed1 msg))
    (cl:cons ':speed2 (speed2 msg))
))
