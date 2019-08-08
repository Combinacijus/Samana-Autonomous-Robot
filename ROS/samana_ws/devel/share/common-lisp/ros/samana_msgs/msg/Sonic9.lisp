; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude Sonic9.msg.html

(cl:defclass <Sonic9> (roslisp-msg-protocol:ros-message)
  ((Header
    :reader Header
    :initarg :Header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (dist1
    :reader dist1
    :initarg :dist1
    :type cl:fixnum
    :initform 0)
   (dist2
    :reader dist2
    :initarg :dist2
    :type cl:fixnum
    :initform 0)
   (dist3
    :reader dist3
    :initarg :dist3
    :type cl:fixnum
    :initform 0)
   (dist4
    :reader dist4
    :initarg :dist4
    :type cl:fixnum
    :initform 0)
   (dist5
    :reader dist5
    :initarg :dist5
    :type cl:fixnum
    :initform 0)
   (dist6
    :reader dist6
    :initarg :dist6
    :type cl:fixnum
    :initform 0)
   (dist7
    :reader dist7
    :initarg :dist7
    :type cl:fixnum
    :initform 0)
   (dist8
    :reader dist8
    :initarg :dist8
    :type cl:fixnum
    :initform 0)
   (dist9
    :reader dist9
    :initarg :dist9
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Sonic9 (<Sonic9>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sonic9>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sonic9)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<Sonic9> is deprecated: use samana_msgs-msg:Sonic9 instead.")))

(cl:ensure-generic-function 'Header-val :lambda-list '(m))
(cl:defmethod Header-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:Header-val is deprecated.  Use samana_msgs-msg:Header instead.")
  (Header m))

(cl:ensure-generic-function 'dist1-val :lambda-list '(m))
(cl:defmethod dist1-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist1-val is deprecated.  Use samana_msgs-msg:dist1 instead.")
  (dist1 m))

(cl:ensure-generic-function 'dist2-val :lambda-list '(m))
(cl:defmethod dist2-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist2-val is deprecated.  Use samana_msgs-msg:dist2 instead.")
  (dist2 m))

(cl:ensure-generic-function 'dist3-val :lambda-list '(m))
(cl:defmethod dist3-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist3-val is deprecated.  Use samana_msgs-msg:dist3 instead.")
  (dist3 m))

(cl:ensure-generic-function 'dist4-val :lambda-list '(m))
(cl:defmethod dist4-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist4-val is deprecated.  Use samana_msgs-msg:dist4 instead.")
  (dist4 m))

(cl:ensure-generic-function 'dist5-val :lambda-list '(m))
(cl:defmethod dist5-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist5-val is deprecated.  Use samana_msgs-msg:dist5 instead.")
  (dist5 m))

(cl:ensure-generic-function 'dist6-val :lambda-list '(m))
(cl:defmethod dist6-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist6-val is deprecated.  Use samana_msgs-msg:dist6 instead.")
  (dist6 m))

(cl:ensure-generic-function 'dist7-val :lambda-list '(m))
(cl:defmethod dist7-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist7-val is deprecated.  Use samana_msgs-msg:dist7 instead.")
  (dist7 m))

(cl:ensure-generic-function 'dist8-val :lambda-list '(m))
(cl:defmethod dist8-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist8-val is deprecated.  Use samana_msgs-msg:dist8 instead.")
  (dist8 m))

(cl:ensure-generic-function 'dist9-val :lambda-list '(m))
(cl:defmethod dist9-val ((m <Sonic9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist9-val is deprecated.  Use samana_msgs-msg:dist9 instead.")
  (dist9 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sonic9>) ostream)
  "Serializes a message object of type '<Sonic9>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'dist1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dist2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dist3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dist4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dist5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dist6)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dist7)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dist8)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dist9)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sonic9>) istream)
  "Deserializes a message object of type '<Sonic9>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist1) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist2) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist3) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist4) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist5) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist6) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist7) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist8) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist9) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sonic9>)))
  "Returns string type for a message object of type '<Sonic9>"
  "samana_msgs/Sonic9")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sonic9)))
  "Returns string type for a message object of type 'Sonic9"
  "samana_msgs/Sonic9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sonic9>)))
  "Returns md5sum for a message object of type '<Sonic9>"
  "9475c4b12aca5b9b235ea1a9e9e22280")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sonic9)))
  "Returns md5sum for a message object of type 'Sonic9"
  "9475c4b12aca5b9b235ea1a9e9e22280")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sonic9>)))
  "Returns full string definition for message of type '<Sonic9>"
  (cl:format cl:nil "std_msgs/Header Header~%int16 dist1~%int16 dist2~%int16 dist3~%int16 dist4~%int16 dist5~%int16 dist6~%int16 dist7~%int16 dist8~%int16 dist9~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sonic9)))
  "Returns full string definition for message of type 'Sonic9"
  (cl:format cl:nil "std_msgs/Header Header~%int16 dist1~%int16 dist2~%int16 dist3~%int16 dist4~%int16 dist5~%int16 dist6~%int16 dist7~%int16 dist8~%int16 dist9~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sonic9>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Header))
     2
     2
     2
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sonic9>))
  "Converts a ROS message object to a list"
  (cl:list 'Sonic9
    (cl:cons ':Header (Header msg))
    (cl:cons ':dist1 (dist1 msg))
    (cl:cons ':dist2 (dist2 msg))
    (cl:cons ':dist3 (dist3 msg))
    (cl:cons ':dist4 (dist4 msg))
    (cl:cons ':dist5 (dist5 msg))
    (cl:cons ':dist6 (dist6 msg))
    (cl:cons ':dist7 (dist7 msg))
    (cl:cons ':dist8 (dist8 msg))
    (cl:cons ':dist9 (dist9 msg))
))
