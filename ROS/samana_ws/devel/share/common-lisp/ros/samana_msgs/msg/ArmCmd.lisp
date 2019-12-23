; Auto-generated. Do not edit!


(cl:in-package samana_msgs-msg)


;//! \htmlinclude ArmCmd.msg.html

(cl:defclass <ArmCmd> (roslisp-msg-protocol:ros-message)
  ((grabber_cmd
    :reader grabber_cmd
    :initarg :grabber_cmd
    :type cl:fixnum
    :initform 0)
   (lifter_cmd
    :reader lifter_cmd
    :initarg :lifter_cmd
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ArmCmd (<ArmCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name samana_msgs-msg:<ArmCmd> is deprecated: use samana_msgs-msg:ArmCmd instead.")))

(cl:ensure-generic-function 'grabber_cmd-val :lambda-list '(m))
(cl:defmethod grabber_cmd-val ((m <ArmCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:grabber_cmd-val is deprecated.  Use samana_msgs-msg:grabber_cmd instead.")
  (grabber_cmd m))

(cl:ensure-generic-function 'lifter_cmd-val :lambda-list '(m))
(cl:defmethod lifter_cmd-val ((m <ArmCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader samana_msgs-msg:lifter_cmd-val is deprecated.  Use samana_msgs-msg:lifter_cmd instead.")
  (lifter_cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmCmd>) ostream)
  "Serializes a message object of type '<ArmCmd>"
  (cl:let* ((signed (cl:slot-value msg 'grabber_cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lifter_cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmCmd>) istream)
  "Deserializes a message object of type '<ArmCmd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'grabber_cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lifter_cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmCmd>)))
  "Returns string type for a message object of type '<ArmCmd>"
  "samana_msgs/ArmCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmCmd)))
  "Returns string type for a message object of type 'ArmCmd"
  "samana_msgs/ArmCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmCmd>)))
  "Returns md5sum for a message object of type '<ArmCmd>"
  "4915ef1fb595c9707da4cb79c7caeeb8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmCmd)))
  "Returns md5sum for a message object of type 'ArmCmd"
  "4915ef1fb595c9707da4cb79c7caeeb8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmCmd>)))
  "Returns full string definition for message of type '<ArmCmd>"
  (cl:format cl:nil "int8 grabber_cmd~%int8 lifter_cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmCmd)))
  "Returns full string definition for message of type 'ArmCmd"
  (cl:format cl:nil "int8 grabber_cmd~%int8 lifter_cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmCmd>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmCmd
    (cl:cons ':grabber_cmd (grabber_cmd msg))
    (cl:cons ':lifter_cmd (lifter_cmd msg))
))
