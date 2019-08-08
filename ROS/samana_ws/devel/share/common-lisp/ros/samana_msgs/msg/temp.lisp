; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude temp.msg.html

(cl:defclass <temp> (roslisp-msg-protocol:ros-message)
  ((temp
    :reader temp
    :initarg :temp
    :type cl:float
    :initform 0.0))
)

(cl:defclass temp (<temp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <temp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'temp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<temp> is deprecated: use samana_msgs-msg:temp instead.")))

(cl:ensure-generic-function 'temp-val :lambda-list '(m))
(cl:defmethod temp-val ((m <temp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:temp-val is deprecated.  Use samana_msgs-msg:temp instead.")
  (temp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <temp>) ostream)
  "Serializes a message object of type '<temp>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <temp>) istream)
  "Deserializes a message object of type '<temp>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temp) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<temp>)))
  "Returns string type for a message object of type '<temp>"
  "samana_msgs/temp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'temp)))
  "Returns string type for a message object of type 'temp"
  "samana_msgs/temp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<temp>)))
  "Returns md5sum for a message object of type '<temp>"
  "b8b64af59df1604df7e647a04e645a45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'temp)))
  "Returns md5sum for a message object of type 'temp"
  "b8b64af59df1604df7e647a04e645a45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<temp>)))
  "Returns full string definition for message of type '<temp>"
  (cl:format cl:nil "float32 temp~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'temp)))
  "Returns full string definition for message of type 'temp"
  (cl:format cl:nil "float32 temp~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <temp>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <temp>))
  "Converts a ROS message object to a list"
  (cl:list 'temp
    (cl:cons ':temp (temp msg))
))
