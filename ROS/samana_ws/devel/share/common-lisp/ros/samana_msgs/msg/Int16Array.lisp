; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude Int16Array.msg.html

(cl:defclass <Int16Array> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (dist
    :reader dist
    :initarg :dist
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Int16Array (<Int16Array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Int16Array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Int16Array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<Int16Array> is deprecated: use samana_msgs-msg:Int16Array instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Int16Array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:header-val is deprecated.  Use samana_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'dist-val :lambda-list '(m))
(cl:defmethod dist-val ((m <Int16Array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:dist-val is deprecated.  Use samana_msgs-msg:dist instead.")
  (dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Int16Array>) ostream)
  "Serializes a message object of type '<Int16Array>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'dist))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Int16Array>) istream)
  "Deserializes a message object of type '<Int16Array>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dist) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dist)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Int16Array>)))
  "Returns string type for a message object of type '<Int16Array>"
  "samana_msgs/Int16Array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Int16Array)))
  "Returns string type for a message object of type 'Int16Array"
  "samana_msgs/Int16Array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Int16Array>)))
  "Returns md5sum for a message object of type '<Int16Array>"
  "ff005e0bae1d9bc418c95faadd17f36d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Int16Array)))
  "Returns md5sum for a message object of type 'Int16Array"
  "ff005e0bae1d9bc418c95faadd17f36d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Int16Array>)))
  "Returns full string definition for message of type '<Int16Array>"
  (cl:format cl:nil "std_msgs/Header header~%int16[] dist~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Int16Array)))
  "Returns full string definition for message of type 'Int16Array"
  (cl:format cl:nil "std_msgs/Header header~%int16[] dist~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Int16Array>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dist) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Int16Array>))
  "Converts a ROS message object to a list"
  (cl:list 'Int16Array
    (cl:cons ':header (header msg))
    (cl:cons ':dist (dist msg))
))
