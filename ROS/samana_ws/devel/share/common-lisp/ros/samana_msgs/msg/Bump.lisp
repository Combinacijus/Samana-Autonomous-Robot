; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude Bump.msg.html

(cl:defclass <Bump> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (bump_bits
    :reader bump_bits
    :initarg :bump_bits
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Bump (<Bump>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bump>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bump)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<Bump> is deprecated: use samana_msgs-msg:Bump instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Bump>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:header-val is deprecated.  Use samana_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'bump_bits-val :lambda-list '(m))
(cl:defmethod bump_bits-val ((m <Bump>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:bump_bits-val is deprecated.  Use samana_msgs-msg:bump_bits instead.")
  (bump_bits m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bump>) ostream)
  "Serializes a message object of type '<Bump>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bump_bits)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bump_bits)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bump>) istream)
  "Deserializes a message object of type '<Bump>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bump_bits)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bump_bits)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bump>)))
  "Returns string type for a message object of type '<Bump>"
  "samana_msgs/Bump")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bump)))
  "Returns string type for a message object of type 'Bump"
  "samana_msgs/Bump")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bump>)))
  "Returns md5sum for a message object of type '<Bump>"
  "be760e4b4932adb10f1e9fc66f2d1769")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bump)))
  "Returns md5sum for a message object of type 'Bump"
  "be760e4b4932adb10f1e9fc66f2d1769")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bump>)))
  "Returns full string definition for message of type '<Bump>"
  (cl:format cl:nil "std_msgs/Header header~%uint16 bump_bits~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bump)))
  "Returns full string definition for message of type 'Bump"
  (cl:format cl:nil "std_msgs/Header header~%uint16 bump_bits~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bump>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bump>))
  "Converts a ROS message object to a list"
  (cl:list 'Bump
    (cl:cons ':header (header msg))
    (cl:cons ':bump_bits (bump_bits msg))
))
